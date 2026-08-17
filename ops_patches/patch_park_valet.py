#!/usr/bin/env python3
# /data/ops_patches/patch_park_valet.py
# "Offroad Without OBD Power" -- auto-offroad the car once it's genuinely
# parked (gear=P, stopped ~3s), and clear that again the moment it leaves
# park, using gear/speed instead of the ignition/OBD power line. Two halves,
# one patcher, one commit: entry lives in starpilot_card.py (already runs
# onroad with carState computed -- zero new subscriptions); exit lives in
# hardwared.py (always-running, since card itself stops once ForceOffroad is
# set). Formerly two separate patchers/commits; merged so they read as one
# feature instead of two unrelated-looking ones.
import os
import sys

CARD = "/data/openpilot/starpilot/controls/starpilot_card.py"
HARDWARED = "/data/openpilot/system/hardware/hardwared.py"

# Anchors are deliberately ONE line each, on a line upstream has no reason to
# touch, and we insert AFTER the anchor + re-emit it. A multi-line anchor dies
# on any insertion inside its span -- that is exactly how Dom decf85985
# (chestnut block in the loop body) and acf32365b (grown __init__) broke this
# stack before.

CARD_OLD_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt\""""
CARD_NEW_INIT = """    self.error_log = ERROR_LOGS_PATH / "error.txt"

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

CARD_OLD_PARKED = """    starpilotCarState.isParked = carState.gearShifter == GearShifter.park"""
CARD_NEW_PARKED = """    starpilotCarState.isParked = carState.gearShifter == GearShifter.park

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

HW_OLD_IMPORTS = """from cereal.services import SERVICE_LIST"""
HW_NEW_IMPORTS = """from cereal.services import SERVICE_LIST
from cereal import car as car_log
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus
from opendbc.car.toyota.values import CAR, DBC
from openpilot.selfdrive.pandad import can_capnp_to_list"""

HW_OLD_PARAMS = """  power_monitor = PowerMonitoring()"""
HW_NEW_PARAMS = """  power_monitor = PowerMonitoring()

  # Park auto-offroad exit detection state (lazy-initialized once CarParams exists).
  PARK_OFFROAD_MARKER = "/data/ops_patches/.park_offroad_forced"
  park_cp = None
  park_gear_msg = None
  park_gear_val = None
  park_can_sock = None"""

HW_OLD_SHOULD_START = "    should_start &= not starpilot_toggles.force_offroad"
HW_NEW_SHOULD_START = "    should_start &= not (starpilot_toggles.force_offroad or params.get_bool(\"ForceOffroad\"))"

# Anchor on the pandaStates read alone, NOT the whole loop head: upstream
# inserts code between `sm.update(...)` and this line (chestnut block, Dom
# decf85985), which broke a multi-line loop-head anchor before. This line is
# unique in the file and our block only needs to run once per iteration ahead
# of the panda handling.
HW_OLD_LOOP = """    pandaStates = sm['pandaStates']"""
HW_NEW_LOOP = """    if park_cp is None and os.path.isfile(PARK_OFFROAD_MARKER):
      try:
        cp_bytes = params.get("CarParams")
        if cp_bytes:
          fp = str(messaging.log_from_bytes(cp_bytes, car_log.CarParams).carFingerprint)
          dbc_path = DBC[fp][Bus.pt] if fp in DBC else DBC[CAR[fp]][Bus.pt]
          can_define = CANDefine(dbc_path)
          for msg in ("GEAR_PACKET_HYBRID", "GEAR_PACKET"):
            gear_vals = can_define.dv.get(msg, {}).get("GEAR")
            if gear_vals:
              for val, text in gear_vals.items():
                if text in ("P", "park"):
                  park_gear_msg, park_gear_val = msg, int(val)
                  break
            if park_gear_msg is not None:
              break
          if park_gear_msg is not None:
            park_cp = CANParser(dbc_path, [(park_gear_msg, float("nan"))], 0)
            park_can_sock = messaging.sub_sock("can", conflate=True)
      except Exception:
        cloudlog.exception("Park exit detector init failed")

    if park_cp is not None and os.path.isfile(PARK_OFFROAD_MARKER):
      try:
        can_strs = messaging.drain_sock_raw(park_can_sock, wait_for_one=False)
        if can_strs:
          park_cp.update(can_capnp_to_list(can_strs))
        gear_ts_ns = park_cp.ts_nanos[park_gear_msg]["GEAR"]
        if gear_ts_ns > 0 and time.monotonic_ns() - gear_ts_ns < 1_000_000_000:
          in_park = park_cp.vl[park_gear_msg]["GEAR"] == park_gear_val
          if not in_park:
            params.put_bool("ForceOffroad", False)
            try:
              os.remove(PARK_OFFROAD_MARKER)
            except FileNotFoundError:
              pass
      except Exception:
        cloudlog.exception("Park exit detector update failed")

    pandaStates = sm['pandaStates']"""


def patch_file(path, replacements, already_marker):
    if not os.path.isfile(path):
        print(f"ERROR: {path} missing")
        return False

    with open(path, "r") as f:
        content = f.read()

    if already_marker in content:
        print(f"{os.path.basename(path)} already patched")
        return True

    for old, new in replacements:
        if content.count(old) != 1:
            print(f"ERROR: anchor found {content.count(old)}x (expected 1) in {os.path.basename(path)}; patch not applied")
            return False
        content = content.replace(old, new, 1)

    with open(path, "w") as f:
        f.write(content)

    print(f"{os.path.basename(path)} patched")
    return True


def main():
    # Entry first, exit second -- if entry's anchors don't match, bail before
    # touching hardwared.py at all (fail fast, no partial-file writes).
    if not patch_file(CARD, [(CARD_OLD_INIT, CARD_NEW_INIT), (CARD_OLD_PARKED, CARD_NEW_PARKED)], "park_offroad_marker"):
        return 1
    if not patch_file(HARDWARED, [
        (HW_OLD_IMPORTS, HW_NEW_IMPORTS),
        (HW_OLD_PARAMS, HW_NEW_PARAMS),
        (HW_OLD_SHOULD_START, HW_NEW_SHOULD_START),
        (HW_OLD_LOOP, HW_NEW_LOOP),
    ], "PARK_OFFROAD_MARKER"):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
