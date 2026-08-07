#!/usr/bin/env python3
# /data/ops_patches/patch_starpilot_card.py
# Entry detection for park-auto-offroad: this hook already runs inside card
# (only_onroad, manager-supervised) with carState already computed -- zero
# new subscriptions, zero new process. Exit is handled separately in
# hardwared.py, since card stops running once ForceOffroad is set.
import os
import sys

CARD = "/data/openpilot/starpilot/controls/starpilot_card.py"

OLD_IMPORT = """from openpilot.starpilot.common.starpilot_variables import ERROR_LOGS_PATH, GearShifter, NON_DRIVING_GEARS"""

NEW_IMPORT = """from pathlib import Path

from openpilot.starpilot.common.starpilot_variables import ERROR_LOGS_PATH, GearShifter, NON_DRIVING_GEARS"""

OLD_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt"

  def handle_button_event"""

NEW_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt"

    # Park auto-offroad entry counters (exit lives in hardwared.py).
    self.park_offroad_counter = 0
    self.park_offroad_threshold = 300  # dwell frames (~3s at 100Hz)
    self.park_offroad_v_ego_threshold = 0.5  # m/s
    self.park_offroad_marker = Path("/data/ops_patches/.park_offroad_forced")

    # Boot-time cleanup of stale marker (ForceOffroad is CLEAR_ON_MANAGER_START).
    try:
      if self.park_offroad_marker.is_file():
        self.park_offroad_marker.unlink()
    except Exception:
      pass

  def handle_button_event"""

OLD_PARKED = """    starpilotCarState.isParked = carState.gearShifter == GearShifter.park
    starpilotCarState.pauseLateral = self.pause_lateral"""

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
          pass

    starpilotCarState.pauseLateral = self.pause_lateral"""


def main():
    if not os.path.isfile(CARD):
        print(f"ERROR: {CARD} missing")
        return 1

    with open(CARD, "r") as f:
        content = f.read()

    if "park_offroad_marker" in content:
        print("starpilot_card.py already patched")
        return 0

    if OLD_IMPORT not in content:
        print("ERROR: expected import line not found in starpilot_card.py; patch not applied")
        return 1
    content = content.replace(OLD_IMPORT, NEW_IMPORT, 1)

    if OLD_INIT not in content:
        print("ERROR: expected __init__ block not found in starpilot_card.py; patch not applied")
        return 1
    content = content.replace(OLD_INIT, NEW_INIT, 1)

    if OLD_PARKED not in content:
        print("ERROR: expected isParked block not found in starpilot_card.py; patch not applied")
        return 1
    content = content.replace(OLD_PARKED, NEW_PARKED, 1)

    with open(CARD, "w") as f:
        f.write(content)

    print("starpilot_card.py patched for park auto-offroad entry")
    return 0


if __name__ == "__main__":
    sys.exit(main())
