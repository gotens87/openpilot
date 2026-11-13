#!/usr/bin/env python3
# Keep C3 Fan at 50% Minimum
#
# comma 3 with a degraded fan: the stock 30% ignition-on floor stalls it.
#
# fan_controller.py: FanController.update() runs the PID with neg_limit = 30
# while ignition is on, so at/below the setpoint the commanded fan duty
# settles at 30% PWM. On this unit the fan motor is worn and will not start
# (or restart after a stall) at 30% duty, so once it stops the device runs
# fanless until the next ignition cycle. Single edit: that one line's
# ignition-on neg_limit is the constant 50. Only that floor changes:
# pos_limit, both ignition-off limits, the setpoint and the feedforward
# curves are untouched, including the new aggressive-cooling curve
# (IS_MICI, aggressive flag), and the PID still modulates freely above the
# floor. Takes effect when manager next starts the hardware stack (each
# boot). No params are written; the fan still turns off when ignition is off.
import os
import sys

HARDWARE_DIR = "/data/openpilot/openpilot/system/hardware"

EDITS = {
  "fan_controller.py": [
    ("""    self.controller.neg_limit = 30 if ignition else 0""",
     """    # Ignition-on floor for a degraded fan that stalls at the stock 30% duty.
    self.controller.neg_limit = 50 if ignition else 0"""),
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
