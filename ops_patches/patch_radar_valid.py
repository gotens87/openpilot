#!/usr/bin/env python3
# /data/ops_patches/patch_radar_valid.py
# Honour the per-track VALID bit on the side clusters.
#
# radar_interface.py's TSS-P Continental path filters only ID==0x3f and
# LONG_DIST<=0, so an invalid side-cluster track is still published as a radar
# point and can be picked up by lead selection / the close-radar veto.
#
# CLUSTER_L/R/L_A/R_A (0x682-0x685) define VALID (bit 6|1@0+) in
# toyota_radar_dsu_tssp.dbc. CLUSTER_F/F_A (0x680/0x681) do NOT, so the front
# clusters -- the lead directly ahead -- are deliberately left ungated. This
# change can only ever DROP junk side tracks; it cannot alter the position or
# scale of any track that was already being published.
#
# Chosen over switching to the OBJECT_0..11 array (0x301-0x317, 12 tracks at
# 16.6 Hz, confirmed live on the bus): OBJECT's LONG_DIST scale is unresolved
# upstream (0.03 vs 0.04) and its DBC LAT_DIST scale disagrees with CLUSTER's on
# an identical bit field (0.018 vs 0.015). A wrong LONG_DIST scale is a wrong
# following distance, so that swap needs a measured scale first.
import os
import sys

RADAR = "/data/openpilot/opendbc_repo/opendbc/car/toyota/radar_interface.py"

OLD_MSGS = """TSSP_CLUSTER_MSGS = list(range(0x680, 0x686))"""

NEW_MSGS = """TSSP_CLUSTER_MSGS = list(range(0x680, 0x686))
# Side clusters publish a per-track VALID bit; the two front clusters do not.
TSSP_CLUSTER_VALID_MSGS = set(range(0x682, 0x686))"""

OLD_FILTER = """      if track_id == 0x3f or cpt["LONG_DIST"] <= 0:"""

NEW_FILTER = """      if ii in TSSP_CLUSTER_VALID_MSGS and not cpt["VALID"]:
        continue
      if track_id == 0x3f or cpt["LONG_DIST"] <= 0:"""


def main():
    if not os.path.isfile(RADAR):
        print(f"ERROR: {RADAR} missing")
        return 1

    with open(RADAR) as f:
        content = f.read()

    if "TSSP_CLUSTER_VALID_MSGS" in content:
        print("radar_interface.py already patched")
        return 0

    if content.count(OLD_MSGS) != 1:
        print(f"ERROR: cluster msg list anchor found {content.count(OLD_MSGS)}x (expected 1) in radar_interface.py; patch not applied")
        return 1
    content = content.replace(OLD_MSGS, NEW_MSGS, 1)

    if content.count(OLD_FILTER) != 1:
        print(f"ERROR: track filter anchor found {content.count(OLD_FILTER)}x (expected 1) in radar_interface.py; patch not applied")
        return 1
    content = content.replace(OLD_FILTER, NEW_FILTER, 1)

    with open(RADAR, "w") as f:
        f.write(content)

    print("radar_interface.py patched to honour side-cluster VALID")
    return 0


if __name__ == "__main__":
    sys.exit(main())
