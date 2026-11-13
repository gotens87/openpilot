#!/usr/bin/env python3
# Ignore Comfort Cap During Lead Takeoff Only (Testing)
#
# The acceleration profile (Eco, Standard, Sport) caps launch at comfort
# levels, for example 2.0 m/s^2 on Standard. During a lead takeoff only,
# that cap is now skipped and the normal takeoff ceiling of 3.5 is used.
#
# Cruise and following still use the profile. All other limits stay: set
# speed, cornering, stop logic. Does nothing if the profile is already at
# 3.5. Tests now check that takeoff goes above the selected cap.
import os
import sys

OPENPILOT_DIR = "/data/openpilot/openpilot"

EDITS = {
  "selfdrive/controls/lib/longitudinal_planner.py": [
    ("""      takeoff_max = accel_ceiling if sm['selfdriveState'].experimentalMode else get_max_accel(v_ego, faster_takeoffs)
      if selected_acceleration is not None and not sm['selfdriveState'].experimentalMode:
        takeoff_max = selected_acceleration
      if cruise_profile is not None:
        takeoff_max = min(takeoff_max, cruise_profile.acceleration_max)
      if selected_acceleration is not None:
        takeoff_max = min(takeoff_max, selected_acceleration)
      a_total_max = np.interp(v_ego, _TAKEOFF_TOTAL_BP, _TAKEOFF_TOTAL_MAX) if faster_takeoffs else np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)""",
     """      takeoff_max = accel_ceiling
      a_total_max = np.interp(v_ego, _TAKEOFF_TOTAL_BP, _TAKEOFF_TOTAL_MAX) if faster_takeoffs else np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)"""),
  ],
  "starpilot/longitudinal/tests/test_follow_takeoff_interaction.py": [
    ("""        assert planner.output_a_target <= .2
        assert planner.last_profile is None
        released |= not planner.output_should_stop and planner.output_a_target == .2""",
     """        if not planner.output_should_stop:
          assert planner.output_a_target > .2
        assert planner.last_profile is None
        released |= not planner.output_should_stop and planner.output_a_target > .2"""),
  ],
  "starpilot/longitudinal/tests/test_lead_takeoff.py": [
    ("""    assert enabled.last_profile is None
    if selected_cap is not None:
      assert enabled.output_a_target <= selected_cap
    if (native.output_should_stop and not enabled.output_should_stop and
        enabled.output_a_target > native.output_a_target and
        enabled.output_a_target >= (.35 if selected_cap is None else selected_cap)):""",
     """    assert enabled.last_profile is None
    if selected_cap is not None and not enabled.output_should_stop:
      assert enabled.output_a_target > selected_cap
    if (native.output_should_stop and not enabled.output_should_stop and
        enabled.output_a_target > native.output_a_target and
        enabled.output_a_target >= (.35 if selected_cap is None else selected_cap)):"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(OPENPILOT_DIR, name)
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
    print("takeoff comfort cap already ignored")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(OPENPILOT_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
