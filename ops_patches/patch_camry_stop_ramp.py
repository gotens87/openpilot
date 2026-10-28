#!/usr/bin/env python3
# /data/ops_patches/patch_camry_stop_ramp.py
# TSS-P Camry hybrid stoppingDecelRate 0.3. Gated on /data/ops_patches/camry_stop_ramp,
# read once at import, default OFF. No marker: stopping_decel_rate matches upstream.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

EDITS = {
  "values.py": [
    ("""import re""",
     """import os
import re"""),
    ("""def stopping_decel_rate(CP):""",
     """_CAMRY_STOP_RAMP = os.path.isfile("/data/ops_patches/camry_stop_ramp")


def stopping_decel_rate(CP):
  if (_CAMRY_STOP_RAMP and CP.carFingerprint == CAR.TOYOTA_CAMRY and
      CP.flags & ToyotaFlags.HYBRID and CP.flags & ToyotaFlags.LONG_FILTER and
      CP.openpilotLongitudinalControl):
    return 0.3"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(TOYOTA, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

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

  if not any(pending.values()):
    print("camry stop ramp already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TOYOTA, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
