#!/usr/bin/env python3
# Toyota Camry Hybrid (TSS-P, openpilot longitudinal): cruise accel onset is
# a hard step the THS eCVT answers fast enough to feel like a kick -- and the
# driver reports SET- / lowering the set speed is just as abrupt as SET+.
#
# longitudinal_planner.py: get_cruise_accel() rate-shapes the cruise request
# with a symmetric slew from J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6] m/s^3 (at
# A_CRUISE_MAX_BP, dt = DT_MDL, 20 Hz), so a +/-1 mph set-speed tap (~0.45
# m/s^2 of request) ramps in ~0.3 s below 10 m/s. a_cruise is only one
# candidate in min(candidates) next to the MPC/lead and e2e sources, so this
# slew shapes the cruise request only. Lead, FCW, model-stop and stop-line
# braking are the MPC/e2e candidates and never pass through J_CRUISE; the
# decel that DOES pass through it is plan-ahead comfort decel -- SET-,
# forceDecel, and the curve / speed-limit / navigation / force-stop / coast
# speed ceilings that clamp v_cruise -- already magnitude-bounded by
# cruise_min (-0.5 stock), and min(candidates) lets any harder MPC/e2e
# demand override it instantly.
#
# With the marker file /data/ops_patches/camry_smooth_onset present (read
# once at import; presence is the whole signal, contents are ignored) the
# POSITIVE side becomes J_CRUISE_UP_VALS = [0.5, 0.5, 0.6, 0.6] m/s^3 (a +1
# tap builds over ~0.9 s at city speeds) and the NEGATIVE side becomes
# J_CRUISE_DOWN_VALS = [0.8, 0.8, 0.8, 0.6] m/s^3 (a -1 tap over ~0.55 s,
# the full -0.5 floor in ~0.6 s): the down side stays firmer than the up
# side so ceiling-approach decel keeps a brisk onset, neither table exceeds
# stock at any speed; the down side matches stock from 25 m/s up, the up side
# is 0.6 from 25 m/s up (stock 0.8 at 25, 0.6 only at 40). Absent marker = both tables are J_CRUISE_VALS = symmetric stock
# slew = stock behavior exactly. Nothing else changes: the accel caps, the
# brake floors (cruise_min, slc_floor, global_floor, profile braking), pitch
# compensation, feedforward, the MPC path and the carcontroller PCM slew
# limits are all untouched, so brake unwind leaving a stop keeps the stock
# 4.0 m/s^2/s up-limit. The marker is import-time state: it takes effect
# when manager next starts the stack (each boot); rm the marker and reboot
# to revert, no rebuild needed. Patch applied + marker absent is the stock
# code path, so marker-off / marker-on reboots are a clean A/B. No params
# are written.
import os
import sys

TREE_DIR = "/data/openpilot"

EDITS = {
  "openpilot/selfdrive/controls/lib/longitudinal_planner.py": [
    ("""import numpy as np""",
     """import os
import numpy as np"""),
    ("""J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]""",
     """J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]

# /data/ops_patches/camry_smooth_onset (read once at import; presence is the
# whole signal): present softens BOTH sides of the cruise accel slew for a
# Camry Hybrid whose eCVT answers the stock 1.6 m/s^3 onset fast enough to
# feel like a kick on SET+ and SET- alike; absent = both tables are
# J_CRUISE_VALS, a symmetric stock slew = stock behavior exactly. The down
# side stays firmer than the up side (0.8 vs 0.5) so ceiling-approach decel
# (curve/SLC/nav/force-stop/coast ceilings, forceDecel -- the only decel
# that passes through this slew; lead/FCW/model-stop braking is the MPC/e2e
# candidates and never does) keeps a brisk onset. Down matches stock from
# 25 m/s up; up is 0.6 from 25 m/s up (softer than stock below 40). The accel caps, brake floors and everything
# downstream are untouched.
CAMRY_SMOOTH_ONSET = os.path.isfile("/data/ops_patches/camry_smooth_onset")
J_CRUISE_UP_VALS = [0.5, 0.5, 0.6, 0.6] if CAMRY_SMOOTH_ONSET else J_CRUISE_VALS
J_CRUISE_DOWN_VALS = [0.8, 0.8, 0.8, 0.6] if CAMRY_SMOOTH_ONSET else J_CRUISE_VALS"""),
    ("""  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))""",
     """  j_cruise_up = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_UP_VALS)
  j_cruise_down = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_DOWN_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise_down * dt, a_cruise_prev + j_cruise_up * dt))"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(TREE_DIR, name)
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
    print("smooth onset already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TREE_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
