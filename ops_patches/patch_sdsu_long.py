#!/usr/bin/env python3
# /data/ops_patches/patch_sdsu_long.py
# Match Reported Vehicle Speed (Testing)
#
# carstate.py: the Camry dash reads about 4% above true speed. Scale
#   vEgoCluster by 1.04 when the car is a Camry so the comma UI matches the
#   dash. Every other Toyota keeps the stock 1.015 factor. The scale runs for
#   every Camry; it no longer depends on a long-filter flag.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

EDITS = {
  "carstate.py": [
    ("""    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars""",
     """    # Camry dash over-reads true speed by ~4%
    if self.CP.carFingerprint == CAR.TOYOTA_CAMRY:
      ret.vEgoCluster = ret.vEgo * 1.04
    else:
      ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars"""),
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
    print("dash speed already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    path = os.path.join(TOYOTA, name)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(content)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
