#!/usr/bin/env python3
# /data/ops_patches/patch_hardwared.py
# Exit detection for park-auto-offroad: card is only_onroad and stops once
# ForceOffroad is set, so this always-running process (hardwared) owns
# clearing it once the car leaves park, via a lightweight raw-CAN gear read.
# Reuses the exact same CANParser pattern already proven working today in
# park_offroad_daemon.py -- not the heavier CarState-class construction.
import os
import sys

HARDWARED = "/data/openpilot/system/hardware/hardwared.py"

OLD_IMPORTS = """import cereal.messaging as messaging
from cereal import log
from cereal.services import SERVICE_LIST"""

NEW_IMPORTS = """import cereal.messaging as messaging
from cereal import car as car_log
from cereal import log
from cereal.services import SERVICE_LIST
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus
from opendbc.car.toyota.values import CAR, DBC
from openpilot.selfdrive.pandad import can_capnp_to_list"""

OLD_PARAMS = """  params = Params()
  power_monitor = PowerMonitoring()"""

NEW_PARAMS = """  params = Params()
  power_monitor = PowerMonitoring()

  # Park auto-offroad exit detection state (lazy-initialized once CarParams exists).
  PARK_OFFROAD_MARKER = "/data/ops_patches/.park_offroad_forced"
  park_cp = None
  park_gear_msg = None
  park_gear_val = None
  park_can_sock = None"""

OLD_SHOULD_START = "    should_start &= not starpilot_toggles.force_offroad"

NEW_SHOULD_START = "    should_start &= not (starpilot_toggles.force_offroad or params.get_bool(\"ForceOffroad\"))"

# Anchor on the pandaStates read alone, NOT the whole loop head: upstream inserts
# code between `sm.update(...)` and this line (chestnut block, Dom decf85985), which
# broke a multi-line loop-head anchor. This line is unique in the file and our block
# only needs to run once per iteration ahead of the panda handling.
OLD_LOOP = """    pandaStates = sm['pandaStates']"""

NEW_LOOP = """    if park_cp is None and os.path.isfile(PARK_OFFROAD_MARKER):
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


def main():
    if not os.path.isfile(HARDWARED):
        print(f"ERROR: {HARDWARED} missing")
        return 1

    with open(HARDWARED, "r") as f:
        content = f.read()

    if "PARK_OFFROAD_MARKER" in content:
        print("hardwared.py already patched")
        return 0

    if content.count(OLD_IMPORTS) != 1:
        print("ERROR: expected imports block not found exactly once in hardwared.py; patch not applied")
        return 1
    content = content.replace(OLD_IMPORTS, NEW_IMPORTS, 1)

    if content.count(OLD_PARAMS) != 1:
        print("ERROR: expected params block not found exactly once in hardwared.py; patch not applied")
        return 1
    content = content.replace(OLD_PARAMS, NEW_PARAMS, 1)

    if content.count(OLD_SHOULD_START) != 1:
        print("ERROR: expected should_start line not found exactly once in hardwared.py; patch not applied")
        return 1
    content = content.replace(OLD_SHOULD_START, NEW_SHOULD_START, 1)

    if content.count(OLD_LOOP) != 1:
        print("ERROR: expected loop start not found exactly once in hardwared.py; patch not applied")
        return 1
    content = content.replace(OLD_LOOP, NEW_LOOP, 1)

    with open(HARDWARED, "w") as f:
        f.write(content)

    print("hardwared.py patched for park auto-offroad exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
