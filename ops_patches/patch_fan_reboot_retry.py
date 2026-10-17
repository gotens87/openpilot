#!/usr/bin/env python3
# comma 3 with an intermittently dead fan (connector/flux): dead episodes read
# 0 rpm the whole drive at 50-100% commanded duty, and the first sign is a
# mid-drive overheat. A full power-off has revived it (93.6C thermal shutdown
# -> next boot 2664 rpm); whether a warm software reboot re-kicks it is
# unknown. This is a mitigation to try, not a fix. UNCONDITIONAL (no marker
# file) so it rides with the git build across reinstalls.
#
# hardwared.py: dead-fan check inside the existing hardware_thread 2 Hz loop --
# no new process, no new cereal subscriber (the msgq 15-reader cap retired the
# standalone-daemon approach). hardwared already has peripheralState
# (fanSpeedRpm) in its SubMaster and computes fanSpeedPercentDesired itself.
#   - arms only when commanded duty >= 50%: fan_floor pins ignition-on duty at
#     50 and the offroad pos_limit is 30, so this never arms offroad-cool and
#     never false-positives on the bench-measured 30% -> 0 rpm stall.
#   - dead = rpm < 500 sustained for 15 s while armed. Healthy fan from a stop
#     at 50% passes 500 rpm by 2 s (864 rpm), so 15 s is >7x margin; 500 rpm
#     matches the fanMalfunction threshold. Reboot is more disruptive than the
#     alarm patch's 5 s grace, so it gets the longer one.
#   - on dead: HARDWARE.reboot() ("sudo reboot"). IsOnroad is not a gate -- a
#     mid-drive stall reboots too; the alarm patch has already been blaring
#     for 10 s by then and overheat would disengage shortly after anyway.
#   - loop guard: consecutive retries are counted in
#     /data/ops_patches/fan_reboot_count (fsync'd before reboot; /data
#     survives the AutomaticUpdates git hard-reset; a Params key would need a
#     C++ rebuild the patch queue can't do). Max 2 retries per dead episode,
#     then the device stays up and usable and patch_fan_alarm's alert covers
#     the drive. The counter resets to 0 the first time the fan is seen
#     spinning (>=500 rpm at >=50% cmd), so the next dead episode gets a
#     fresh pair of retries.
import os
import sys

HARDWARE_DIR = "/data/openpilot/openpilot/system/hardware"

EDITS = {
  "hardwared.py": [
    ("""OFFROAD_DANGER_TEMP = 85 if HARDWARE.get_device_type() == "mici" else 75""",
     """OFFROAD_DANGER_TEMP = 85 if HARDWARE.get_device_type() == "mici" else 75


# --- ops_patches: dead-fan reboot retry (see ops_patches/patch_fan_reboot_retry.py) ---
FAN_REBOOT_MIN_CMD = 50    # arm only when commanded duty >= this (%); offroad cap is 30, so never arms offroad-cool
FAN_REBOOT_RPM = 500       # below this rpm the fan counts as not spinning (matches fanMalfunction threshold)
FAN_DEAD_GRACE_S = 15.     # sustained dead time before rebooting; healthy fan passes 500 rpm by 2 s from a stop at 50%
FAN_REBOOT_MAX = 2         # max consecutive reboot retries per dead episode
FAN_REBOOT_COUNTER = "/data/ops_patches/fan_reboot_count"


def _read_fan_reboot_count() -> int:
  try:
    with open(FAN_REBOOT_COUNTER) as f:
      return max(0, int(f.read().strip()))
  except (OSError, ValueError):
    return 0


def _write_fan_reboot_count(n: int) -> None:
  try:
    if n <= 0:
      os.remove(FAN_REBOOT_COUNTER)
      return
    with open(FAN_REBOOT_COUNTER, "w") as f:
      f.write(str(n))
      f.flush()
      os.fsync(f.fileno())
  except OSError:
    pass
"""),
    ("""  pwrsave = False
  offroad_cycle_count = 0""",
     """  pwrsave = False
  offroad_cycle_count = 0

  # ops_patches: dead-fan reboot retry state
  fan_bad_since: float | None = None
  fan_reboot_requested = False
  fan_gave_up = False
  fan_dead_reboots = _read_fan_reboot_count()"""),
    ("""    if show_alert:
      msg.deviceState.fanSpeedPercentDesired = 100""",
     """    if show_alert:
      msg.deviceState.fanSpeedPercentDesired = 100

    # *** ops_patches: dead-fan reboot retry ***
    fan_rpm = peripheralState.fanSpeedRpm if (sm.alive["peripheralState"] and sm.valid["peripheralState"]) else None
    fan_cmd = msg.deviceState.fanSpeedPercentDesired
    if fan_rpm is None or fan_cmd < FAN_REBOOT_MIN_CMD:
      fan_bad_since = None  # disarmed: panda not reporting, or duty below the 50% arm threshold
    elif fan_rpm >= FAN_REBOOT_RPM:
      fan_bad_since = None
      fan_gave_up = False
      if fan_dead_reboots > 0:
        fan_dead_reboots = 0
        _write_fan_reboot_count(0)
        cloudlog.event("fan spinning, dead-fan reboot counter reset", rpm=fan_rpm, cmd=fan_cmd)
    else:
      if fan_bad_since is None:
        fan_bad_since = time.monotonic()
      elif not fan_reboot_requested and (time.monotonic() - fan_bad_since) > FAN_DEAD_GRACE_S:
        if fan_dead_reboots < FAN_REBOOT_MAX:
          fan_dead_reboots += 1
          fan_reboot_requested = True
          _write_fan_reboot_count(fan_dead_reboots)
          cloudlog.event("fan dead, rebooting to retry", rpm=fan_rpm, cmd=fan_cmd,
                         attempt=fan_dead_reboots, max_attempts=FAN_REBOOT_MAX, error=True)
          os.sync()
          try:
            HARDWARE.reboot("fan_dead")
          except Exception:
            cloudlog.exception("fan-dead reboot failed")
        elif not fan_gave_up:
          fan_gave_up = True
          cloudlog.event("fan dead, reboot retries exhausted, staying up", rpm=fan_rpm, cmd=fan_cmd,
                         attempts=fan_dead_reboots, error=True)"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(HARDWARE_DIR, name)
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
    print("fan reboot retry already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(HARDWARE_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
