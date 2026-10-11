#!/usr/bin/env python3
# /data/ops_patches/patch_radar_lat_sign.py
# Stop negating LAT_DIST on the TSS-P Continental radar path.
#
# On the 2018 Camry TSS-P (Continental radar), the operator confirmed on-road
# that published radar points are left/right mirrored: a car on the right
# shows on the left in the radar point display. The Continental path negates
# the raw LAT_DIST sign; the adjacent yvRel = float(cpt["LAT_SPEED"]) is
# already un-negated, so dropping the minus makes position and lateral speed
# share the same raw sign convention.
#
# The Denso path elsewhere in the file uses a different syntax
# (self.pts[ii].yRel = -cpt['LAT_DIST']) and is deliberately left untouched;
# the anchor below matches only the Continental float(cpt[...]) form and
# appears exactly once.
import os
import sys

RADAR = "/data/openpilot/opendbc_repo/opendbc/car/toyota/radar_interface.py"

OLD_YREL = """      self.pts[track_id].yRel = -float(cpt["LAT_DIST"])"""

NEW_YREL = """      self.pts[track_id].yRel = float(cpt["LAT_DIST"])"""


def main():
    if not os.path.isfile(RADAR):
        print(f"ERROR: {RADAR} missing")
        return 1

    with open(RADAR) as f:
        content = f.read()

    if NEW_YREL in content:
        print("radar_interface.py already patched")
        return 0

    if content.count(OLD_YREL) != 1:
        print(f"ERROR: yRel anchor found {content.count(OLD_YREL)}x (expected 1) in radar_interface.py; patch not applied")
        return 1
    content = content.replace(OLD_YREL, NEW_YREL, 1)

    with open(RADAR, "w") as f:
        f.write(content)

    print("radar_interface.py patched to keep raw LAT_DIST sign on TSS-P Continental path")
    return 0


if __name__ == "__main__":
    sys.exit(main())
