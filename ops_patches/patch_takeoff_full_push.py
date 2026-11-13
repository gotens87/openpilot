#!/usr/bin/env python3
# Keep Full Launch Push to 29 km/h (Testing)
#
# Faster Lead Takeoff used to switch itself off at 7 km/h and gave its full push
# only while the car was still sitting stopped, so the car went soft right after
# it started rolling.
#
# Now it stays active up to 29 km/h and keeps the full push for 2.5 seconds after
# the car starts moving, including the first 7 km/h. It still drops the push if
# you are closing on the lead, the gap gets tight, or the lead brakes.
#
# Tests updated, plus a new test that the push holds below 7 km/h.
import os
import sys

LONGITUDINAL_DIR = "/data/openpilot/openpilot/starpilot/longitudinal"

EDITS = {
  "lead_takeoff.py": [
    ("""BREAKAWAY_MIN_DISTANCE = 3.75
BREAKAWAY_MIN_LEAD_SPEED = .6""",
     """BREAKAWAY_MIN_DISTANCE = 3.75
BREAKAWAY_MIN_LEAD_SPEED = .6
LAUNCH_MAX_SPEED = 8.0
BREAKAWAY_HOLD_NS = 2_500_000_000"""),
    ("""        not 0 <= f.speed <= 2.0 or f.mpc_accel < 0 or f.acceleration_max <= 0 or not .5 <= f.follow_seconds <= 3 or""",
     """        not 0 <= f.speed <= LAUNCH_MAX_SPEED or f.mpc_accel < 0 or f.acceleration_max <= 0 or not .5 <= f.follow_seconds <= 3 or"""),
    ("""    if floor is None and self.hold_floor is not None and f.now_ns <= self.hold_until:
      if (lead.distance >= 3.75 and lead.acceleration >= -.2 and max(f.speed - lead.speed, 0.) <= .45 and
          lead.distance / max(f.speed, .001) - f.follow_seconds >= .1):
        floor = self.hold_floor""",
     """    if self.hold_floor is not None and f.now_ns <= self.hold_until:
      if (lead.distance >= 3.75 and lead.acceleration >= -.2 and max(f.speed - lead.speed, 0.) <= .45 and
          lead.distance / max(f.speed, .001) - f.follow_seconds >= .1):
        floor = max(floor or 0., self.hold_floor)"""),
    ("""      result = max(result, f.acceleration_max)
    if self.previous_should_stop and not output_stop and release_ready:""",
     """      result = max(result, f.acceleration_max)
      self.hold_floor, self.hold_until = f.acceleration_max, f.now_ns + BREAKAWAY_HOLD_NS
    if self.previous_should_stop and not output_stop and release_ready:"""),
  ],
  "tests/test_lead_takeoff.py": [
    ("""  assert LeadTakeoff().step(True, frame(leads=(departing,), acceleration_max=2., blocked=True), 0., True) == (0., True)


def test_creep_confirmation_and_profile_cap_remain_effective():""",
     """  assert LeadTakeoff().step(True, frame(leads=(departing,), acceleration_max=2., blocked=True), 0., True) == (0., True)


def test_breakaway_hold_survives_speeds_below_departure_floor_gate():
  departing = replace(frame().leads[0], distance=5., speed=2.5, acceleration=.5)
  policy = LeadTakeoff()
  assert policy.step(True, frame(leads=(departing,), acceleration_max=2.), .2, False) == (2., False)
  for tick, speed in enumerate((.3, 1.0, 1.9), start=1):
    assert policy.step(True, frame(tick, leads=(departing,), acceleration_max=2.,
                                   standstill=False, speed=speed), .2, False) == (2., False)
  close = replace(departing, distance=3.6)
  dropped = policy.step(True, frame(4, leads=(close,), acceleration_max=2., standstill=False, speed=1.9), .2, False)
  assert dropped[0] < 1. and dropped[1] is False


def test_creep_confirmation_and_profile_cap_remain_effective():"""),
    ("""{'speed': 2.01}""",
     """{'speed': 8.01}"""),
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
    print("full launch push already applied")
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
