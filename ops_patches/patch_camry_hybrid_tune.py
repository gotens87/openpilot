#!/usr/bin/env python3
# /data/ops_patches/patch_camry_hybrid_tune.py
# Camry Hybrid longitudinal tune, split into independent parts. Every part is
# gated behind its own runtime marker under /data/ops_patches/, read once at
# process start (a reboot applies a change), and defaults OFF; with no
# markers the car runs the generic LONG_FILTER tune untouched. No part
# depends on another part's marker, so any single part flips with a file +
# reboot, no code change.
#
# markers (all /data/ops_patches/<name>):
#   camry_ki        THS hybrid ki tune
#   camry_unwind    8x integral-mismatch unwind
#   camry_coast     permit-braking coast band
#   camry_signflip  no-lead cruise sign-flip limiter
#   camry_stop_ramp stoppingDecelRate 0.3
#   camry_hybrid_ff 0.8 positive feedforward scale
#   camry_accel_cap hold ACCEL_MAX at 1.5 (panda allows 2.0 either way)
#
# values.py: camry_part(CP, part) helper (TOYOTA_CAMRY + HYBRID + LONG_FILTER
#   scoping, marker existence cached once at import) plus the camry_stop_ramp
#   gate in stopping_decel_rate and the camry_accel_cap gate in
#   CarControllerParams.
# carcontroller.py: camry_ki in the ki selection; camry_unwind on the
#   integral-mismatch unwind; camry_coast on the permit-braking coast band
#   and camry_signflip on the no-lead sign-flip limiter (both reused from
#   prius_longitudinal, not duplicated); camry_hybrid_ff alone on the 0.8
#   positive feedforward scale.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

EDITS = {
  "values.py": [
    ("""import re""",
     """import os
import re"""),
    ("""  def __init__(self, CP):
    if CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT:
      self.ACCEL_MAX = 2.0""",
     """  def __init__(self, CP):
    if CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and not camry_part(CP, "camry_accel_cap"):
      self.ACCEL_MAX = 2.0"""),
    ("""def stopping_decel_rate(CP):""",
     """# camry hybrid tune parts: each part is on iff /data/ops_patches/<part>
# exists, checked once at import (touch the file and reboot to flip a part)
CAMRY_MARKERS = {part: os.path.isfile(f"/data/ops_patches/{part}") for part in
                 ("camry_ki", "camry_unwind", "camry_coast", "camry_signflip",
                  "camry_stop_ramp", "camry_hybrid_ff", "camry_accel_cap")}


def camry_part(CP, part) -> bool:
  return bool(CAMRY_MARKERS[part] and CP.carFingerprint == CAR.TOYOTA_CAMRY and
              CP.flags & ToyotaFlags.HYBRID and CP.flags & ToyotaFlags.LONG_FILTER)


def stopping_decel_rate(CP):
  if camry_part(CP, "camry_stop_ramp") and CP.openpilotLongitudinalControl:
    return 0.3"""),
  ],
  "carcontroller.py": [
    ("""from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaFlags, TOYOTA_AUTO_HOLD_CARS, uses_toyota_auto_hold_aeb""",
     """from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaFlags, TOYOTA_AUTO_HOLD_CARS, camry_part, uses_toyota_auto_hold_aeb"""),
    ("""  if prius or CP.flags & ToyotaFlags.TSS2:""",
     """  if prius or camry_part(CP, "camry_ki") or CP.flags & ToyotaFlags.TSS2:"""),
    ("""                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):""",
     """                       rate=1 / (DT_CTRL * 3))


CAMRY_HYBRID_POSITIVE_FEEDFORWARD_SCALE = 0.8


def get_camry_hybrid_feedforward(accel):
  return accel * CAMRY_HYBRID_POSITIVE_FEEDFORWARD_SCALE if accel > 0.0 else accel


class CarController(CarControllerBase):"""),
    ("""    self.prius_longitudinal = prius_longitudinal.enabled(CP)""",
     """    self.prius_longitudinal = prius_longitudinal.enabled(CP)
    self.camry_unwind = camry_part(CP, "camry_unwind")
    self.camry_coast = camry_part(CP, "camry_coast")
    self.camry_signflip = camry_part(CP, "camry_signflip")
    self.camry_hybrid_ff = camry_part(CP, "camry_hybrid_ff")"""),
    ("""          unwind = ACCEL_PID_UNWIND * (8. if self.prius_longitudinal and pcm_accel_cmd * self.long_pid.i < 0. else 1.)""",
     """          unwind = ACCEL_PID_UNWIND * (8. if (self.prius_longitudinal or self.camry_unwind) and pcm_accel_cmd * self.long_pid.i < 0. else 1.)"""),
    ("""          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=(prius_longitudinal.get_prius_feedforward(pcm_accel_cmd, CS.out.vEgo)
                                                            if self.prius_longitudinal else pcm_accel_cmd),
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)""",
     """          feedforward = pcm_accel_cmd
          if self.prius_longitudinal:
            feedforward = prius_longitudinal.get_prius_feedforward(feedforward, CS.out.vEgo)
          elif self.camry_hybrid_ff:
            # the THS eCVT over-responds to positive requests, scale them down
            feedforward = get_camry_hybrid_feedforward(feedforward)

          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=feedforward,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)"""),
    ("""        if self.prius_longitudinal:
          self.permit_braking = prius_longitudinal.update_permit_braking(
            self.permit_braking, net_acceleration_request_min, stopping, long_active, CS.out.vEgo, lead)""",
     """        if self.prius_longitudinal or self.camry_coast:
          self.permit_braking = prius_longitudinal.update_permit_braking(
            self.permit_braking, net_acceleration_request_min, stopping, long_active, CS.out.vEgo, lead)"""),
    ("""        if self.prius_longitudinal:
          pcm_accel_cmd = prius_longitudinal.limit_no_lead_cruise_sign_flip(
            pcm_accel_cmd, actuators.accel, stopping, CS.out.vEgo, CS.out.cruiseState.speed, lead)
          pcm_accel_cmd = prius_longitudinal.limit_prius_stopping_accel(
            pcm_accel_cmd, actuators.accel, stopping, CS.out.vEgo, lead)""",
     """        if self.prius_longitudinal or self.camry_signflip:
          pcm_accel_cmd = prius_longitudinal.limit_no_lead_cruise_sign_flip(
            pcm_accel_cmd, actuators.accel, stopping, CS.out.vEgo, CS.out.cruiseState.speed, lead)
        if self.prius_longitudinal:
          pcm_accel_cmd = prius_longitudinal.limit_prius_stopping_accel(
            pcm_accel_cmd, actuators.accel, stopping, CS.out.vEgo, lead)"""),
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
    print("camry hybrid tune already applied")
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
