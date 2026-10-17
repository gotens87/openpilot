#!/usr/bin/env python3
# comma 3 with an intermittently dead fan, an operator on noise-cancelling
# headphones, and the fan_floor patch pinning ignition-on duty at 50%: the
# stock fanMalfunction alert is both disarmed and silent. UNCONDITIONAL (no
# marker file) so it rides with the git build across the operator's frequent
# reinstalls.
#
# selfdrived.py:
#   - arm at fanSpeedPercentDesired >= 50 (was > 50): with fan_floor active,
#     desired == 50 at cool temps and the stock check never arms -- one failed
#     drive climbed to 87.6 C before anything showed. Without the floor
#     (stock 30% duty) this changes nothing in practice.
#   - grace 15.0 s -> 5.0 s. From a stop at 50% duty a healthy fan reads 135
#     rpm at 1 s, 864 at 2 s, 1282 at 3 s; a dead fan reads 0 all drive. 5 s
#     is >2x the worst-case healthy spin-up past the 500 rpm threshold with
#     zero false-positive risk, and fan.h has no stall-retry worth waiting
#     for (its cooldown counter is tach-line noise blanking). The stale
#     "5 seconds" comment becomes true. rpm < 500 threshold unchanged.
#
# events.py: fanMalfunction ET.PERMANENT becomes a full-screen userPrompt
#   Alert ("TAKE CONTROL" / "Fan Failure: Device May Overheat and Disengage")
#   with AudibleAlert.warningSoft: loops while the event is active, force-
#   ramps to MAX_VOLUME in 4 s and escalates to dm_critical_max after 8 s
#   (soundd.py), under WarningSoftVolume (operator default; EngageVolume=0
#   would mute an engage-based pick, promptRepeat never ramps). Priority.HIGH:
#   above routine LOW/MID alerts, below the HIGHEST genuine disengage/
#   collision alerts. PERMANENT only -- no state transition added; overheat
#   already disengages at its thermal band.
import os
import sys

SELFDRIVED_DIR = "/data/openpilot/openpilot/selfdrive/selfdrived"

EDITS = {
  "selfdrived.py": [
    ("""    # Alert if fan isn't spinning for 5 seconds""",
     """    # Alert if fan isn't spinning for 5 seconds (healthy fan at 50% duty from a stop passes 500 rpm by 2 s)"""),
    ("""      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:""",
     """      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired >= 50:"""),
    ("""        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:""",
     """        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:"""),
  ],
  "events.py": [
    ("""  # Thrown when the fan is driven at >50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Fan Malfunction", "Contact comma.ai/support"),
  },""",
     """  # Thrown when the fan is driven at >=50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: Alert(
      "TAKE CONTROL",
      "Fan Failure: Device May Overheat and Disengage",
      AlertStatus.userPrompt, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningSoft, .2),
  },"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(SELFDRIVED_DIR, name)
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
    print("fan alarm already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(SELFDRIVED_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
