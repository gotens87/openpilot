#!/usr/bin/env python3
# comma 3 with a degraded fan: the stock 30% ignition-on floor stalls it.
#
# fan_controller.py: FanController.update() runs the PID with neg_limit = 30
# while ignition is on, so at/below the 75C setpoint the commanded fan duty
# settles at 30% PWM. On this unit the fan motor is worn and will not start
# (or restart after a stall) at 30% duty, so once it stops the device runs
# fanless until the next ignition cycle. With the marker file
# /data/ops_patches/fan_floor present (read once at import) the ignition-on
# neg_limit floor becomes int(contents) clamped to 30..100 (empty or
# unparseable contents = 50). Absent marker = None = stock behavior exactly
# (30). Only the ignition-on neg_limit changes: pos_limit, both ignition-off
# limits, the setpoint and the feedforward curve are untouched, and the PID
# still modulates freely above the floor. The marker is import-time state:
# it takes effect when manager next starts the hardware stack (each boot).
# No params are written; the fan still turns off when ignition is off.
import os
import sys

HARDWARE_DIR = "/data/openpilot/openpilot/system/hardware"

EDITS = {
  "fan_controller.py": [
    ("""import numpy as np""",
     """import os
import numpy as np"""),
    ("""OFFSET = 0 if HARDWARE.get_device_type() == "mici" else 5""",
     """OFFSET = 0 if HARDWARE.get_device_type() == "mici" else 5


# /data/ops_patches/fan_floor (read once at import) raises the ignition-on
# neg_limit floor for a degraded fan that stalls at the stock 30% duty:
# int(contents) clamped to 30..100, empty/unparseable = 50, absent = None
# = stock 30. pos_limit, the ignition-off limits, setpoint and feedforward
# are untouched.
FAN_FLOOR_MARKER = "/data/ops_patches/fan_floor"


def _read_fan_floor():
  if not os.path.isfile(FAN_FLOOR_MARKER):
    return None
  try:
    with open(FAN_FLOOR_MARKER) as f:
      return min(100, max(30, int(f.read())))
  except (OSError, ValueError):
    return 50


FAN_FLOOR = _read_fan_floor()"""),
    ("""    self.controller.neg_limit = 30 if ignition else 0""",
     """    self.controller.neg_limit = (FAN_FLOOR or 30) if ignition else 0"""),
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
    print("fan floor already applied")
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
