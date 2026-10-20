#!/usr/bin/env python3
# Settings drift: an update/reinstall can reset user params (/data/params/d)
# and wipe feature markers (/data/ops_patches), and the car then boots on the
# wrong tune until someone notices.
#
# This patcher installs openpilot/starpilot/ops_settings.py -- ONE desired
# settings list -- and hooks it into manager.py: manager_init() calls
# enforce_ops_settings() right after its own default-params loop (so the
# list wins) and before any managed process starts (markers are read at
# import by controls processes, so they take effect the same boot).
#
#  - Params: enforced to the exact desired value every boot; drift is
#    corrected and logged.
#  - Markers: created ONLY if missing, never overwritten (the owner hand-
#    tunes values like camry_cruise_shaper); markers not in the list mean
#    "off" and are left absent.
#  - Driving model: the small-profile selection in /data/models/<gen>/
#    preferences.json is enforced through the model manager's own "active"
#    action (its locking + artifact verification), never by raw-writing
#    JSON. If the desired artifact is not on disk, enforcement logs and
#    leaves the selection alone -- never switches to a missing model.
#  - Cloud provider: enforced through connect_provider.select_provider
#    (its own lock + revision check). A change only records `selected`;
#    activate_at_boot (already in manager_init, runs earlier) applies it
#    on the NEXT boot. No reboot is forced and enforcement cannot loop.
#  - Every correction goes to cloudlog and is appended, timestamped, to
#    /data/ops_patches/settings_drift.log. No drift = nothing logged.
#  - The manager.py call is wrapped in try/except: enforcement can never
#    block boot.
#
# The list lives inside the build, not in /data/ops_patches, because
# /data/ops_patches itself can get wiped. Adding a setting later is a
# one-line edit to DESIRED_PARAMS / DESIRED_MARKERS / DESIRED_MODEL /
# DESIRED_PROVIDER in the module below (edit it here, so CI rebuilds keep it).
import os
import sys

MANAGER_DIR = "/data/openpilot/openpilot/system/manager"
STARPILOT_DIR = "/data/openpilot/openpilot/starpilot"

# raw string: the module source contains a \n escape that must be written
# verbatim to ops_settings.py, not expanded in this patcher.
OPS_SETTINGS_PY = r'''"""Ops desired-settings list, enforced once per boot from manager_init.

Single source of truth for operator settings that must survive updates and
reinstalls: a fixed set of user params, the /data/ops_patches feature
markers, the small-profile driving model, and the cloud provider.
manager_init calls enforce_ops_settings() after its default-params loop
(so this list wins) and before any managed process starts: markers and the
model selection are read at process start, so they apply the same boot; a
cloud provider change is only recorded here and applied by
activate_at_boot on the following boot, by design of the provider module.
The call site wraps enforcement in try/except, so nothing here can block
boot.

Adding a setting later = one line in DESIRED_PARAMS, DESIRED_MARKERS,
DESIRED_MODEL, or DESIRED_PROVIDER.
"""
import os
import time

from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import atomic_write

MARKER_DIR = "/data/ops_patches"
DRIFT_LOG = os.path.join(MARKER_DIR, "settings_drift.log")

# Param key -> exact desired value, in the same typed representation this
# fork's Params API uses: get() and get_default_value() both return typed
# values (bool for type-1 keys, int for type-2 keys), and put() accepts
# those types directly -- manager_init's default loop passes
# get_default_value's output straight to put(). Comparing typed values
# against get() means no drift -> no write -> no log.
DESIRED_PARAMS = {
  "IsMetric": True,
  "ExperimentalMode": False,
  "EngageVolume": 0,
  "ForceStops": False,
  "FasterLeadTakeoff": True,
  "LongitudinalPersonality": 0,
  "CameraView": 0,
}

# Marker name -> contents. Created ONLY if the marker is absent and never
# overwritten afterwards: the owner hand-tunes values like camry_cruise_shaper
# and those must survive enforcement. Markers not listed here mean "off"
# and are left absent (never created, never deleted).
DESIRED_MARKERS = {
  "camry_cem": "",
  "camry_cluster_speed": "1.04",
  "camry_coast": "",
  "camry_cruise_shaper": "",
  "camry_hybrid_ff": "",
  "camry_ki": "",
  "camry_signflip": "",
  "camry_sng": "",
  "camry_stop_ramp": "",
  "camry_unwind": "",
  "fan_floor": "50",
}

# Markers that must stay ABSENT: the loop above only creates, never
# deletes, so a retired marker goes here and is removed if present (logged
# like any other drift). camry_small_step is superseded by
# camry_cruise_shaper.
DESIRED_ABSENT_MARKERS = ("camry_small_step",)

# Non-param settings, enforced through their owner modules' own APIs:
# DESIRED_MODEL is a small-profile model id from the starpilot model
# catalog. Its artifact must already be downloaded; enforcement never
# downloads and never switches to a model whose artifact is missing (the
# car would have no model) -- it logs and leaves the selection alone.
# DESIRED_PROVIDER is a provider id from connect_provider.PROVIDERS. A
# change is recorded via select_provider and applied by activate_at_boot
# on the NEXT boot; no reboot is forced and enforcement converges instead
# of looping.
DESIRED_MODEL = "rdf63"
DESIRED_PROVIDER = "konik"


def _log_drift(lines):
  for line in lines:
    cloudlog.warning("Ops settings drift corrected: %s", line)
  try:
    with open(DRIFT_LOG, "a") as f:
      for line in lines:
        f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} {line}\n")
  except OSError:
    cloudlog.exception("Ops settings: could not append to drift log")


def _enforce_model(drift):
  from openpilot.starpilot.models.manager import ModelManager, ModelError, artifact_path, preferences

  have = preferences().get("small")
  if have == DESIRED_MODEL:
    return
  try:
    path = artifact_path(DESIRED_MODEL)
  except ModelError:
    path = None
  if path is None or path.is_symlink() or not path.is_file():
    cloudlog.warning("Ops settings: desired model %s artifact not on disk; leaving small=%r", DESIRED_MODEL, have)
    return
  # parked/gpu callables: at manager_init no managed process is running yet,
  # so the selection cannot disturb a live drive (modeld reads it at process
  # start, later this boot); gpu presence is irrelevant for the small profile
  # and the lambda avoids importing modeld helpers at boot.
  mgr = ModelManager(parked=lambda: True, gpu_present=lambda: False)
  try:
    mgr.action("active", {"profile": "small", "model": DESIRED_MODEL})
  finally:
    mgr.close()
  drift.append(f"model small: {have!r} -> {DESIRED_MODEL!r}")


def _enforce_provider(drift):
  from openpilot.starpilot.connect.provider import select_provider, status

  st = status()
  active, selected = st["active"], st["selected"]
  if selected == DESIRED_PROVIDER:
    # No actionable drift: either active already matches, or a switch to the
    # desired provider is already pending and activate_at_boot applies it
    # next boot. Nothing to write; this can never loop.
    return
  select_provider(DESIRED_PROVIDER, st["revision"], lambda: True)
  note = "active already matches" if active == DESIRED_PROVIDER else "applies next boot"
  drift.append(f"cloud provider: selected {selected!r} -> {DESIRED_PROVIDER!r} (active {active!r}; {note})")


def enforce_ops_settings(params):
  drift = []

  for key, want in DESIRED_PARAMS.items():
    try:
      have = params.get(key)
      if have != want:
        params.put(key, want, block=True)
        drift.append(f"param {key}: {have!r} -> {want!r}")
    except Exception:
      cloudlog.exception("Ops settings: failed to enforce param %s", key)

  os.makedirs(MARKER_DIR, exist_ok=True)
  for name, contents in DESIRED_MARKERS.items():
    try:
      path = os.path.join(MARKER_DIR, name)
      if os.path.lexists(path):
        continue
      with atomic_write(path, "w", overwrite=True) as f:
        f.write(contents)
      drift.append(f"marker {name}: absent -> {contents!r}")
    except Exception:
      cloudlog.exception("Ops settings: failed to create marker %s", name)

  for name in DESIRED_ABSENT_MARKERS:
    try:
      path = os.path.join(MARKER_DIR, name)
      if not os.path.lexists(path):
        continue
      os.remove(path)
      drift.append(f"marker {name}: present -> absent")
    except Exception:
      cloudlog.exception("Ops settings: failed to remove marker %s", name)

  try:
    _enforce_model(drift)
  except Exception:
    cloudlog.exception("Ops settings: failed to enforce driving model")
  try:
    _enforce_provider(drift)
  except Exception:
    cloudlog.exception("Ops settings: failed to enforce cloud provider")

  # no drift -> nothing is logged, anywhere
  if drift:
    _log_drift(drift)
'''

EDITS = {
  "manager.py": [
    ("""    default_value = params.get_default_value(k)
    if default_value is not None and not os.path.lexists(params.get_param_path(k)):
      params.put(k, default_value, block=True)

  # Create folders needed for msgq""",
     """    default_value = params.get_default_value(k)
    if default_value is not None and not os.path.lexists(params.get_param_path(k)):
      params.put(k, default_value, block=True)

  # Ops desired-settings enforcement: once per boot, after the default loop
  # above so the list wins, before any managed process starts (markers are
  # read at import by controls processes). Wrapped so it can never block boot.
  try:
    from openpilot.starpilot.ops_settings import enforce_ops_settings
    enforce_ops_settings(params)
  except Exception:
    cloudlog.exception("Ops settings enforcement failed; continuing boot")

  # Create folders needed for msgq"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(MANAGER_DIR, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

  if not os.path.isdir(STARPILOT_DIR):
    print(f"ERROR: {STARPILOT_DIR} missing")
    return 1

  # validate every anchor before writing anything
  pending = {}
  for name, edits in EDITS.items():
    content = contents[name]
    todo = []
    for old, new in edits:
      if new in content:
        continue
      if content.count(old) != 1:
        print(f"ERROR: anchor found {content.count(old)}x (expected 1) in {name}; patch not applied")
        print(f"  anchor: {old.splitlines()[0]!r}")
        return 1
      todo.append((old, new))
    pending[name] = todo

  module_path = os.path.join(STARPILOT_DIR, "ops_settings.py")
  module_pending = True
  if os.path.isfile(module_path):
    with open(module_path) as f:
      module_pending = f.read() != OPS_SETTINGS_PY

  if not any(pending.values()) and not module_pending:
    print("ops settings already applied")
    return 0

  for name, todo in pending.items():
    if not todo:
      continue
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(MANAGER_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  if module_pending:
    with open(module_path, "w") as f:
      f.write(OPS_SETTINGS_PY)
    print(f"installed {module_path}")

  return 0


if __name__ == "__main__":
  sys.exit(main())
