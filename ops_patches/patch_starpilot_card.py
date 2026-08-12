#!/usr/bin/env python3
# /data/ops_patches/patch_starpilot_card.py
# Entry detection for park-auto-offroad: this hook already runs inside card
# (only_onroad, manager-supervised) with carState already computed -- zero
# new subscriptions, zero new process. Exit is handled separately in
# hardwared.py, since card stops running once ForceOffroad is set.
import os
import sys

CARD = "/data/openpilot/starpilot/controls/starpilot_card.py"

# Anchors are deliberately ONE line each, on a line upstream has no reason to touch.
# A multi-line anchor dies on any insertion inside its span (Dom acf32365b grew
# __init__; decf85985 inserted into a loop body). Insert AFTER the anchor and re-emit
# it, so trailing context is never part of the match.
#
# No import anchor: `Path` is imported lazily inside __init__ instead of patching the
# module import block. That block is a long, frequently-edited import list, and the
# only place Path is needed is the marker construction below.

OLD_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt\""""

NEW_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt"

    # Park auto-offroad entry counters (exit lives in hardwared.py).
    from pathlib import Path
    self.park_offroad_counter = 0
    self.park_offroad_threshold = 300  # dwell frames (~3s at 100Hz)
    self.park_offroad_v_ego_threshold = 0.5  # m/s
    self.park_offroad_marker = Path("/data/ops_patches/.park_offroad_forced")

    # Boot-time cleanup of stale marker (ForceOffroad is CLEAR_ON_MANAGER_START).
    try:
      if self.park_offroad_marker.is_file():
        self.park_offroad_marker.unlink()
    except Exception:
      pass"""

OLD_PARKED = """    starpilotCarState.isParked = carState.gearShifter == GearShifter.park"""

NEW_PARKED = """    starpilotCarState.isParked = carState.gearShifter == GearShifter.park

    # Auto-force-offroad when genuinely parked and stopped (entry only).
    parked_and_stopped = starpilotCarState.isParked and abs(carState.vEgo) < self.park_offroad_v_ego_threshold
    if parked_and_stopped:
      self.park_offroad_counter += 1
    else:
      self.park_offroad_counter = 0

    if self.park_offroad_counter >= self.park_offroad_threshold and not self.park_offroad_marker.is_file():
      if not self.params.get_bool("ForceOffroad"):
        self.params.put_bool("ForceOffroad", True)
        try:
          self.park_offroad_marker.parent.mkdir(parents=True, exist_ok=True)
          self.park_offroad_marker.write_text("1")
        except Exception:
          pass"""


def main():
    if not os.path.isfile(CARD):
        print(f"ERROR: {CARD} missing")
        return 1

    with open(CARD, "r") as f:
        content = f.read()

    if "park_offroad_marker" in content:
        print("starpilot_card.py already patched")
        return 0

    if content.count(OLD_INIT) != 1:
        print(f"ERROR: error_log anchor found {content.count(OLD_INIT)}x (expected 1) in starpilot_card.py; patch not applied")
        return 1
    content = content.replace(OLD_INIT, NEW_INIT, 1)

    if content.count(OLD_PARKED) != 1:
        print(f"ERROR: isParked anchor found {content.count(OLD_PARKED)}x (expected 1) in starpilot_card.py; patch not applied")
        return 1
    content = content.replace(OLD_PARKED, NEW_PARKED, 1)

    with open(CARD, "w") as f:
        f.write(content)

    print("starpilot_card.py patched for park auto-offroad entry")
    return 0


if __name__ == "__main__":
    sys.exit(main())
