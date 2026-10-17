#!/usr/bin/env python3
# "Offroad Without OBD Power" -- park auto-offroad for a constantly powered
# comma (2018 Camry Hybrid, TSS-P, ignition never drops at park). Force the
# device offroad once genuinely parked (gear=P, |vEgo| < 0.5 m/s, disengaged,
# ~3s dwell) and release it the moment the gear leaves P. On 7.0 both halves
# go through the native drive-state owner (snapshot + request with
# expected_revision, authorized/override_allowed callbacks) instead of the
# removed ForceOffroad param: entry requests "offroad" from card.py (runs
# onroad, carState in hand, no new subscribers), release requests "auto" from
# hardwared.py (always runs, card stops once offroad) by parsing
# GEAR_PACKET(_HYBRID) off CAN like before.
#
# Callback semantics that matter here: DriveStateOwner.request() rejects ANY
# Offroad request whose override_allowed() returns False ("Park the vehicle
# and disengage before forcing Offroad"). override_allowed means "this caller
# vouches for the parked/disengaged physical evidence" -- the local UI button
# passes True (control.py DriveStateControl.change), and the entry gate below
# IS our physical evidence, so it must pass True as well. Passing False makes
# every entry attempt raise Rejected and the marker is never written.
#
# The marker file carries the DriveStateSession id, so we only ever release a
# force we applied this manager session -- a stale marker after reboot is just
# deleted (manager re-initializes the request to auto, so nothing stays stuck),
# and a user's manual drive-state choice is never touched.
import os
import sys

CARD = "/data/openpilot/openpilot/selfdrive/car/card.py"
HARDWARED = "/data/openpilot/openpilot/system/hardware/hardwared.py"

CARD_OLD_INIT = "    self.rk = Ratekeeper(100, print_delay_threshold=None)"
CARD_NEW_INIT = """    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # Park auto-offroad entry state (release half lives in hardwared.py).
    self.park_offroad_counter = 0
    self.park_offroad_owner = None
    try:
      from openpilot.starpilot.drive_state.owner import DriveStateOwner
      from openpilot.starpilot.storage import starpilot_storage_root
      self.park_offroad_owner = DriveStateOwner(self.params, starpilot_storage_root() / "drive-state")
    except Exception:
      cloudlog.exception("park auto-offroad init failed")"""

CARD_OLD_PARK = "    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)"
CARD_NEW_PARK = """    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    # Force offroad once genuinely parked (gear P, |vEgo| < 0.5 m/s, disengaged,
    # 300 frames ~3s at 100Hz); retry at most once per 3s while parked so a
    # failing request can't spam every frame.
    # override_allowed=True is required: owner.request() rejects any Offroad
    # request whose override_allowed() is False. The callback asserts "this
    # caller vouches for the parked/disengaged evidence" -- our gate above IS
    # that evidence, exactly like the local UI button passing True in
    # control.py. It does NOT mean "user override permitted".
    try:
      parked = (CS.gearShifter == structs.CarState.GearShifter.park and
                abs(CS.vEgo) < 0.5 and not self.sm['carControl'].enabled)
      self.park_offroad_counter = self.park_offroad_counter + 1 if parked else 0
      if (self.park_offroad_counter > 0 and self.park_offroad_counter % 300 == 0 and
          self.park_offroad_owner is not None):
        session = self.params.get("DriveStateSession")
        sid = session.get("id") if isinstance(session, dict) else None
        marker = "/data/ops_patches/.park_offroad_forced"
        try:
          with open(marker) as f:
            claimed = f.read().strip() == sid
        except OSError:
          claimed = False
        if sid is not None and not claimed:
          snap = self.park_offroad_owner.snapshot()
          if snap.available and snap.mode == "auto":
            req = self.park_offroad_owner.request("offroad", expected_revision=snap.revision,
                                                  authorized=lambda: True, override_allowed=lambda: True)
            if req.available and req.mode == "offroad":
              os.makedirs("/data/ops_patches", exist_ok=True)
              with open(marker, "w") as f:
                f.write(sid)
    except Exception:
      cloudlog.exception("park auto-offroad entry failed")"""

HW_OLD_INIT = "  power_monitor = PowerMonitoring()"
HW_NEW_INIT = """  power_monitor = PowerMonitoring()

  # Park auto-offroad release state (CAN parser is lazy-built on first need).
  park_offroad_marker = "/data/ops_patches/.park_offroad_forced"
  park_cp = None
  park_gear_msg = None
  park_gear_val = None
  park_can_sock = None"""

# Anchor on the pandaStates read alone (single unique line); the release block
# must run every loop iteration, ahead of the 2Hz-gated body below it.
HW_OLD_LOOP = "    pandaStates = sm['pandaStates']"
HW_NEW_LOOP = """    # Release park auto-offroad the moment the gear leaves P (runs while offroad).
    if os.path.isfile(park_offroad_marker):
      try:
        with open(park_offroad_marker) as f:
          park_sid = f.read().strip()
        session = params.get("DriveStateSession")
        sid = session.get("id") if isinstance(session, dict) else None
        if sid is None or park_sid != sid:
          try:
            os.remove(park_offroad_marker)  # stale claim from a dead session
          except OSError:
            pass
        else:
          if park_cp is None:
            from opendbc.can import CANDefine, CANParser
            from opendbc.car import Bus
            from opendbc.car.structs import car as car_log
            from opendbc.car.toyota.values import CAR, DBC
            from openpilot.selfdrive.pandad import can_capnp_to_list
            cp_bytes = params.get("CarParams")
            if cp_bytes:
              fp = str(messaging.log_from_bytes(cp_bytes, car_log.CarParams).carFingerprint)
              dbc_path = DBC[fp][Bus.pt] if fp in DBC else DBC[CAR[fp]][Bus.pt]
              can_define = CANDefine(dbc_path)
              for msg in ("GEAR_PACKET_HYBRID", "GEAR_PACKET"):
                for val, text in (can_define.dv.get(msg, {}).get("GEAR") or {}).items():
                  if text in ("P", "park"):
                    park_gear_msg, park_gear_val = msg, int(val)
                    break
                if park_gear_msg is not None:
                  break
              if park_gear_msg is not None:
                park_cp = CANParser(dbc_path, [(park_gear_msg, float("nan"))], 0)
                park_can_sock = messaging.sub_sock("can", conflate=True)
          if park_cp is not None:
            can_strs = messaging.drain_sock_raw(park_can_sock, wait_for_one=False)
            if can_strs:
              park_cp.update(can_capnp_to_list(can_strs))
            gear_ts_ns = park_cp.ts_nanos[park_gear_msg]["GEAR"]
            if (gear_ts_ns > 0 and time.monotonic_ns() - gear_ts_ns < 1_000_000_000 and
                park_cp.vl[park_gear_msg]["GEAR"] != park_gear_val):
              snap = drive_state.snapshot()
              if snap.available:
                if snap.mode == "offroad":
                  drive_state.request("auto", expected_revision=snap.revision,
                                      authorized=lambda: True, override_allowed=lambda: False)
                try:
                  os.remove(park_offroad_marker)
                except OSError:
                  pass
      except Exception:
        cloudlog.exception("park auto-offroad release failed")

    pandaStates = sm['pandaStates']"""

CARD_REPLACEMENTS = [(CARD_OLD_INIT, CARD_NEW_INIT), (CARD_OLD_PARK, CARD_NEW_PARK)]
HW_REPLACEMENTS = [(HW_OLD_INIT, HW_NEW_INIT), (HW_OLD_LOOP, HW_NEW_LOOP)]


def main():
  # Verify every anchor in every target BEFORE writing any file (no partials).
  plan = []
  for path, replacements, marker in ((CARD, CARD_REPLACEMENTS, "park_offroad_owner"),
                                     (HARDWARED, HW_REPLACEMENTS, "park_offroad_marker")):
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path, "r") as f:
      content = f.read()
    if marker in content:
      print(f"{os.path.basename(path)} already patched")
      continue
    for old, _ in replacements:
      n = content.count(old)
      if n != 1:
        print(f"ERROR: anchor found {n}x (expected 1) in {os.path.basename(path)}; patch not applied")
        return 1
    plan.append((path, replacements, content))

  for path, replacements, content in plan:
    for old, new in replacements:
      content = content.replace(old, new, 1)
    with open(path, "w") as f:
      f.write(content)
    print(f"{os.path.basename(path)} patched")
  return 0


if __name__ == "__main__":
  sys.exit(main())
