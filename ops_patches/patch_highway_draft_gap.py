#!/usr/bin/env python3
# Highway Draft Gap: Follow Closer at 90+ km/h
#
# At matched speed the MPC's zero-cost gap is t_follow*v + STOP_DISTANCE; the
# 6 m STOP_DISTANCE term is a stop-line buffer that also sits behind a settled
# highway lead. long_mpc.update() moves the lead obstacle 2.5 m farther (MPC
# settles 2.5 m closer), only while all hold: v_ego >= 25 m/s (90 km/h),
# |v_ego - v_lead| <= 1 m/s, the lead's predicted speed over the horizon never
# drops more than 1 m/s below v_ego, and the lead is no closer than
# t_follow*v + STOP_DISTANCE - 2.5 - 1 m (1 m below the trimmed target, so the
# trimmed equilibrium keeps its own gate on). A braking lead, a stopped lead, a
# cut-in closer than that, and the stop line keep the stock obstacle. Takes
# effect when the planner process next starts; no params are written, no
# acados cost or model change.
import os
import sys

MPC_PATH = "/data/openpilot/openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"

OLD_CONST = "STOP_DISTANCE = 6.0\n"
NEW_CONST = """STOP_DISTANCE = 6.0
_DRAFT_M = 2.5


def _draft_obstacle(obstacle, lead_xv, v_ego, t_follow):
  if v_ego < 25.0:
    return obstacle
  v_lead = lead_xv[:, 1]
  if abs(v_ego - v_lead[0]) > 1.0 or v_ego - v_lead.min() > 1.0:
    return obstacle
  if lead_xv[0, 0] < t_follow * v_ego + STOP_DISTANCE - _DRAFT_M - 1.0:
    return obstacle
  return obstacle + _DRAFT_M
"""

OLD_OBS = (
    "    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])\n"
    "    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])\n"
)
NEW_OBS = (
    "    lead_0_obstacle = _draft_obstacle(lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1]), lead_xv_0, self.x0[1], t_follow)\n"
    "    lead_1_obstacle = _draft_obstacle(lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1]), lead_xv_1, self.x0[1], t_follow)\n"
)

EDITS = [(OLD_CONST, NEW_CONST), (OLD_OBS, NEW_OBS)]


def main():
  if not os.path.isfile(MPC_PATH):
    print(f"ERROR: {MPC_PATH} missing")
    return 1
  with open(MPC_PATH) as f:
    content = f.read()

  todo = []
  for old, new in EDITS:
    if new in content:
      continue
    if content.count(old) != 1:
      print(f"ERROR: anchor found {content.count(old)}x (expected 1) in long_mpc.py; patch not applied")
      print(f"  anchor: {old.splitlines()[0]!r}")
      return 1
    todo.append((old, new))

  if not todo:
    print("highway draft gap already applied")
    return 0

  for old, new in todo:
    content = content.replace(old, new, 1)
  with open(MPC_PATH, "w") as f:
    f.write(content)
  print(f"patched long_mpc.py ({len(todo)} edits)")
  return 0


if __name__ == "__main__":
  sys.exit(main())
