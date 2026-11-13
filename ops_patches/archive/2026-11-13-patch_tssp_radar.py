#!/usr/bin/env python3
# /data/ops_patches/patch_tssp_radar.py
# TSS-P Camry: parse the Continental radar instead of running vision-only ACC.
#
# values.py + ClusterRadarInterface are upstream now. Remaining:
# interface.py: only expect radar traffic when the filter hardware is present.
# cluster_radar.py: keep LAT_DIST raw sign; the negated form mirrors on this car.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

# patch_sdsu_long.py must run first: the radarUnavailable gate uses camry_sdsu
DEPENDENCIES = {"interface.py": "camry_sdsu = candidate == CAR.TOYOTA_CAMRY"}

EDITS = {
  "interface.py": [
    ("""    if candidate == CAR.TOYOTA_CAMRY:
      ret.radarUnavailable = False
      ret.deprecated.radarTimeStep = 0.1""",
     """    if candidate == CAR.TOYOTA_CAMRY:
      ret.radarUnavailable = not camry_sdsu
      ret.deprecated.radarTimeStep = 0.1"""),
  ],
  "cluster_radar.py": [
    ("""      point.yRel = -float(cluster['LAT_DIST'])""",
     """      point.yRel = float(cluster['LAT_DIST'])  # raw sign; negated form mirrors on this Camry"""),
  ],
}


def main():
  contents = {}
  for name in set(EDITS) | set(DEPENDENCIES):
    path = os.path.join(TOYOTA, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

  for name, marker in DEPENDENCIES.items():
    if marker not in contents[name]:
      print(f"ERROR: {name} is missing {marker!r}; run patch_sdsu_long.py first")
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

  if not any(pending.values()):
    print("tssp radar already applied")
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
