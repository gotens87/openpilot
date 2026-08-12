#!/usr/bin/env python3
# /data/ops_patches/patch_radar_yaw.py
# Tunable radar boresight (yaw) correction for the TSS-P Continental path.
#
# Why: radarState.leadOne.yRel measured against the vision lead over 12209
# straight-road frames (5 drives, July 2026) sits LEFT by a margin that grows
# with range: +0.023 m @14 m, +0.055 @25 m, +0.075 @35 m, +0.072 @65 m,
# +0.104 @87 m. Growing-with-range is the signature of an ANGULAR error, i.e.
# the radar is aimed a fraction of a degree off the car's centreline. Mean
# implied angle over the positive bins is +0.095 deg.
#
# openpilot applies no boresight correction on this path, and this car's radar
# alignment has never been physically measured. A bracket angle cannot be
# recovered from a log, so expose it as a knob the operator can dial on-road.
#
# DEFAULT IS 0.0 => byte-for-byte identical behaviour to stock. Setting the
# param applies yRel -= tan(angle) * dRel, which is a pure lateral correction:
# it cannot change dRel, vRel, or which messages are consumed, so following
# distance and braking are untouched at any value.
#
#   set:    echo 0.1 > /data/ops_patches/radar_yaw_deg   (then reboot)
#   read at process start only -> reboot (or restart radard) to apply.
import os
import sys

RADAR = "/data/openpilot/opendbc_repo/opendbc/car/toyota/radar_interface.py"

OLD_CONST = """TSSP_RADAR_EGO_SPEED_SCALE = 0.922"""

NEW_CONST = """TSSP_RADAR_EGO_SPEED_SCALE = 0.922
# Radar boresight yaw correction, degrees, read from a plain file. Positive value
# shifts reported lead position to the RIGHT. Absent/empty file = 0.0 = stock.
# A file rather than a Param because Params rejects keys absent from
# params_keys.h, which would make the knob silently inert.
TSSP_RADAR_YAW_FILE = "/data/ops_patches/radar_yaw_deg\""""

OLD_INIT = """    self.radar_acc_tssp = CP.carFingerprint in RADAR_ACC_TSSP_CAR"""

NEW_INIT = """    self.radar_acc_tssp = CP.carFingerprint in RADAR_ACC_TSSP_CAR

    # Boresight correction, read once at process start. Any failure -> 0.0 (stock).
    self._radar_yaw_tan = 0.0
    try:
      import math
      with open(TSSP_RADAR_YAW_FILE) as _f:
        self._radar_yaw_tan = math.tan(math.radians(float(_f.read().strip())))
    except Exception:
      self._radar_yaw_tan = 0.0"""

OLD_YREL = """      self.pts[track_id].yRel = -float(cpt["LAT_DIST"])"""

NEW_YREL = """      self.pts[track_id].yRel = -float(cpt["LAT_DIST"]) - self._radar_yaw_tan * float(cpt["LONG_DIST"])"""


def main():
    if not os.path.isfile(RADAR):
        print(f"ERROR: {RADAR} missing")
        return 1

    with open(RADAR) as f:
        content = f.read()

    if "TSSP_RADAR_YAW_FILE" in content:
        print("radar_interface.py already has yaw correction")
        return 0

    for old, new, label in ((OLD_CONST, NEW_CONST, "ego-speed const"),
                            (OLD_INIT, NEW_INIT, "radar_acc_tssp init"),
                            (OLD_YREL, NEW_YREL, "yRel assignment")):
        if content.count(old) != 1:
            print(f"ERROR: {label} anchor found {content.count(old)}x (expected 1) in radar_interface.py; patch not applied")
            return 1
        content = content.replace(old, new, 1)

    with open(RADAR, "w") as f:
        f.write(content)

    print("radar_interface.py patched with tunable boresight yaw correction")
    return 0


if __name__ == "__main__":
    sys.exit(main())
