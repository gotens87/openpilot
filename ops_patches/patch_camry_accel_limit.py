#!/usr/bin/env python3
# Restore 2.0 Launch Accel on Radar-Filter Camry
#
# Radar-filter Camry skips LONG_FILTER, so RAISED_ACCEL_LIMIT never sets and ACCEL_MAX stays 1.5.
# Extend that one condition for this car; every other car is unchanged.
import os
import sys

TOYOTA_DIR = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

EDITS = {
  "interface.py": [
    ("""    if ret.flags & ToyotaFlags.TSS2 or ret.flags & ToyotaFlags.LONG_FILTER and not (rav4h_sdsu or eps8_filter or legacy_filter):""",
     """    if ret.flags & ToyotaFlags.TSS2 or ret.flags & ToyotaFlags.LONG_FILTER and not (rav4h_sdsu or eps8_filter or legacy_filter) or (candidate == CAR.TOYOTA_CAMRY and radar_filter):"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(TOYOTA_DIR, name)
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

  if not any(pending.values()):
    print("camry accel limit already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TOYOTA_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
