#!/usr/bin/env python3
# Firmer Get-Ready Push Before Lead Moves (Testing)
#
# Behind a stopped lead that is about to go, the car used to wait with a tiny
# push (0.18 to 0.35 m/s^2). The hybrid engine needs about 0.5 to wake up, so
# the car woke up late and lost the light.
#
# Now the get-ready push is 1.0 (lead close or clearly leaving) and 0.6 (lead
# just starting to creep). The engine is awake and the car leans forward as the
# lead goes. It still backs off if the gap gets tight.
#
# Tests updated for the new numbers.
import os
import sys

LONGITUDINAL_DIR = "/data/openpilot/openpilot/starpilot/longitudinal"

EDITS = {
  "lead_takeoff.py": [
    ("""    if close_ready or regular:
      floor = .35
    elif creep_ready:
      floor = .18""",
     """    if close_ready or regular:
      floor = 1.0
    elif creep_ready:
      floor = .6"""),
  ],
  "tests/test_lead_takeoff.py": [
    ("""  assert policy.step(True, frame(7), 0., True) == (.35, False)""",
     """  assert policy.step(True, frame(7), 0., True) == (1., False)"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(LONGITUDINAL_DIR, name)
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
    print("get-ready push already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(LONGITUDINAL_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
