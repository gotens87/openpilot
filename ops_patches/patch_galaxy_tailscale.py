#!/usr/bin/env python3
# Galaxy Tailscale card in the Galaxy settings page.
#
# Five new files (TailscaleSettings, HTTP/JS tests, the card) live in
# $OPS_PATCH_DIR/galaxy_tailscale/ mirroring checkout paths. apply-patchers.sh
# stages this patcher away from ops_patches/ (sed rewrite, no __file__ to
# siblings) and sets OPS_PATCH_DIR to that dir. Missing env or payload dir
# is a hard error. For each payload: target absent -> copy, identical ->
# skip, present and different -> ERROR (upstream may have added its own; a
# human decides). Then eight anchor edits (five in server.py, three in
# galaxy.js) hook GET/POST /api/galaxy/tailscale and mount <GalaxyTailscale>.
# NEW already present -> skip; OLD must occur exactly once. Every payload
# conflict and every anchor is validated before any write. Idempotent:
# identical payloads + all NEW present = already applied, exit 0.
import os
import sys

REPO = "/data/openpilot"

PAYLOADS = (
  "openpilot/starpilot/galaxy/tailscale.py",
  "openpilot/starpilot/galaxy/tests/test_tailscale.py",
  "openpilot/starpilot/galaxy/tests/test_tailscale_http.py",
  "openpilot/starpilot/galaxy/web/js/tailscale.js",
  "openpilot/starpilot/galaxy/tests/test_tailscale.mjs",
)

EDITS = {
  "openpilot/starpilot/galaxy/server.py": [
    ("""                local_access=None, tmux_live=None, starport_logs=None, starport_log_sender=None, hotspot=None, toyota_tools=None,""",
     """                local_access=None, tmux_live=None, starport_logs=None, starport_log_sender=None, hotspot=None, tailscale=None, toyota_tools=None,"""),
    ("""  hotspot_source = hotspot
  hotspot_source_lock = threading.Lock()

  def hotspot_owner():
    nonlocal hotspot_source
    with hotspot_source_lock:
      if hotspot_source is None:
        from openpilot.starpilot.galaxy.hotspot import HotspotSettings
        hotspot_source = HotspotSettings()
    return hotspot_source""",
     """  hotspot_source = hotspot
  hotspot_source_lock = threading.Lock()
  tailscale_source = tailscale
  tailscale_source_lock = threading.Lock()

  def hotspot_owner():
    nonlocal hotspot_source
    with hotspot_source_lock:
      if hotspot_source is None:
        from openpilot.starpilot.galaxy.hotspot import HotspotSettings
        hotspot_source = HotspotSettings()
    return hotspot_source

  def tailscale_owner():
    nonlocal tailscale_source
    with tailscale_source_lock:
      if tailscale_source is None:
        from openpilot.starpilot.galaxy.tailscale import TailscaleSettings
        tailscale_source = TailscaleSettings()
    return tailscale_source"""),
    ("""      elif path in ('/api/galaxy/session', '/api/galaxy/session/qr.svg'):""",
     """      elif path == '/api/galaxy/tailscale':
        if not self.require_session():
          return
        try:
          result = tailscale_owner().snapshot(parked())
        except (OSError, ValueError):
          self.json(503, {'error': 'Tailscale settings are unavailable.'})
        else:
          if self.require_session():
            self.json(200, result)
      elif path in ('/api/galaxy/session', '/api/galaxy/session/qr.svg'):"""),
    ("""                      '/api/galaxy/pair', '/api/galaxy/unpair', '/api/galaxy/device-name', '/api/galaxy/hotspot',\n                      '/api/cameras/snapshot',""",
     """                      '/api/galaxy/pair', '/api/galaxy/unpair', '/api/galaxy/device-name', '/api/galaxy/hotspot', '/api/galaxy/tailscale',\n                      '/api/cameras/snapshot',"""),
    ("""      if path == '/api/recordings/action':""",
     """      if path == '/api/galaxy/tailscale':
        identity = self.settings_session()
        if identity is None:
          self.json(401, {'error': 'Sign in to Galaxy'})
          return
        if type(payload) is not dict or payload.get('action') != 'connect':
          self.json(400, {'error': 'Invalid action'})
          return
        try:
          with effect_lock:
            result = tailscale_owner().connect(parked)
        except PermissionError:
          self.json(409, {'error': 'Park before changing Tailscale.'})
        except FileExistsError:
          self.json(409, {'error': 'Tailscale is already working.'})
        else:
          if self.settings_session() == identity:
            self.json(202, result)
          else:
            self.json(401, {'error': 'Sign in to Galaxy'})
        return
      if path == '/api/recordings/action':"""),
  ],
  "openpilot/starpilot/galaxy/web/js/galaxy.js": [
    ("""import { GalaxyCompanion, newerCompanionVersion } from "./companion.js"
import { GalaxyHotspot } from "./hotspot.js"
import { LocalAccess } from "./local-access.js""",
     """import { GalaxyCompanion, newerCompanionVersion } from "./companion.js"
import { GalaxyHotspot } from "./hotspot.js"
import { GalaxyTailscale } from "./tailscale.js"
import { LocalAccess } from "./local-access.js"""),
    ("""  components: { GxState, GxNotice, BluetoothNotice, InstallApp, GalaxyHotspot, GalaxyCompanion, LocalAccess },""",
     """  components: { GxState, GxNotice, BluetoothNotice, InstallApp, GalaxyHotspot, GalaxyTailscale, GalaxyCompanion, LocalAccess },"""),
    ("""      </ol>
      <GalaxyHotspot :mode="mode" :unauthorized="unauthorized" />
      <GalaxyCompanion v-if="!ios" :mode="mode" :unauthorized="unauthorized" @status="companionStatus = $event" />""",
     """      </ol>
      <GalaxyHotspot :mode="mode" :unauthorized="unauthorized" />
      <GalaxyTailscale :mode="mode" :unauthorized="unauthorized" />
      <GalaxyCompanion v-if="!ios" :mode="mode" :unauthorized="unauthorized" @status="companionStatus = $event" />"""),
  ],
}


def main():
  patch_dir = os.environ.get("OPS_PATCH_DIR")
  if not patch_dir:
    print("ERROR: OPS_PATCH_DIR unset")
    return 1
  payload_dir = os.path.join(patch_dir, "galaxy_tailscale")
  if not os.path.isdir(payload_dir):
    print(f"ERROR: {payload_dir} missing")
    return 1

  payload_bytes = {}
  copies = []
  for rel in PAYLOADS:
    src = os.path.join(payload_dir, rel)
    if not os.path.isfile(src):
      print(f"ERROR: {src} missing")
      return 1
    with open(src, "rb") as f:
      data = f.read()
    payload_bytes[rel] = data
    dst = os.path.join(REPO, rel)
    if not os.path.isfile(dst):
      copies.append(rel)
      continue
    with open(dst, "rb") as f:
      existing = f.read()
    if existing != data:
      print(f"ERROR: {dst} exists and differs from payload; patch not applied")
      return 1

  contents = {}
  for name in EDITS:
    path = os.path.join(REPO, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

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

  if not copies and not any(pending.values()):
    print("galaxy tailscale already applied")
    return 0

  for rel in copies:
    dst = os.path.join(REPO, rel)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "wb") as f:
      f.write(payload_bytes[rel])
    print(f"copied {rel}")

  for name, todo in pending.items():
    if not todo:
      continue
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(REPO, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
