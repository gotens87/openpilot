"""Ops desired-settings list, enforced once per boot from manager_init.

Single source of truth for operator settings that must survive updates and
reinstalls: a fixed set of user params, the small-profile driving model,
and the cloud provider.
manager_init calls enforce_ops_settings() after its default-params loop
(so this list wins) and before any managed process starts: the model
selection is read at process start, so it applies the same boot; a
cloud provider change is only recorded here and applied by
activate_at_boot on the following boot, by design of the provider module.
The call site wraps enforcement in try/except, so nothing here can block
boot.

Adding a setting later = one line in DESIRED_PARAMS, DESIRED_MODEL, or
DESIRED_PROVIDER.
"""
import os
import time

from openpilot.common.swaglog import cloudlog

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
  # "ForceStops": False,  # owner may want it later
  "FasterLeadTakeoff": False,
  "LongitudinalPersonality": 0,
  "CameraView": 0,
  "CustomPersonalities": True,
  "AggressiveFollow": 1.25,
  "AggressiveFollowHigh": 0.5,
  "AlphaLongitudinalEnabled": True,
  "SNGHack": True,
  "SpeedLimitController": True,
  "SLCPriority1": "Map Data",
  "SLCPriority2": "Dashboard",
  "SLCConfirmation": 0,
  "SLCOffsetSchedule": {
    "version": 1,
    "bounds_mps": [0.0, 8.1, 13.6, 16.4, 21.9, 24.9, 38.9, 44.4],
    "offsets_mps": [0.0, 0.0, 0.0, 0.0, 0.0, 10 / 3.6, 0.0],
  },
}

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
    os.makedirs(MARKER_DIR, exist_ok=True)
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
