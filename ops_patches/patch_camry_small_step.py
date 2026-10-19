#!/usr/bin/env python3
# 2018 Camry Hybrid (THS eCVT): a +1 mph set-speed tap makes get_cruise_accel
# request ~0.45-0.55 m/s^2 for ~0.5 s; the pulse crosses the THS engine-start
# threshold from EV mode (0 -> ~1500 rpm, off again within ~3 s). Softening
# only the jerk slew does not help: the peak request is unchanged.
#
# longitudinal_planner.py: with /data/ops_patches/camry_small_step present
# (read once at import), get_cruise_accel clamps only its proportional
# target's bounds for the driver's own small set-speed gaps:
# |v_cruise - v_ego| <= 2.2 m/s (~5 mph) -> cap = interp(v_ego, [0,10,25,40],
# [0.35,0.32,0.27,0.22]) m/s^2 (~0.30 at 16 m/s, below the observed <0.4-0.5
# engine-start threshold), applied symmetrically for up and down taps;
# 2.2..3.6 m/s of gap linearly blends both bounds back to the stock-computed
# cruise_min/max_accel; beyond 3.6 m/s (~8 mph) stock exactly. The cap is
# FLAT vs gap, not gap-proportional: FrogPilot/Dom-style ramp-off ceilings
# (cap -> 0 as gap -> 0) halve the effective gain near set speed and hold the
# request in the EV/regen zero-torque dither band (vault: "agitation"/judder
# on the Dom fork until a floor was added; ramp-off also exceeds the
# engine-start threshold for any gap above ~+2 mph). Here the cap IS the
# floor: for |gap| <= cap the request equals the stock proportional term, so
# near-set-speed behavior is bit-identical to stock, and the 2.2..3.6 blend
# is a ramp-with-a-floor (0.30 -> stock max), never a collapse. The blended
# bounds are clamped to never exceed the stock bounds, so the throttle-gate
# coast limit and profile caps always win. Optional int marker contents =
# flat cap in cm/s^2 clamped 15..60 replacing the speed table (e.g. 22
# reproduces the Dom +1 mph ceiling without its collapse); empty or
# unparseable = default table; absent marker = None = stock exactly. The cap
# is gated off whenever any cruise/curve/navigation ceiling, force-stop plan,
# coast plan, forceDecel, standstill or driver pedal is active, and applies
# only to the cruise candidate: MPC/lead, e2e, shouldStop, the jerk slew,
# allowThrottle and all ceiling logic are untouched, and min() candidate
# arbitration can never be weakened by a gentler cruise candidate. The marker
# is import-time state: it takes effect when manager next starts the controls
# stack (each boot). No params are written.
import os
import sys

PLANNER_DIR = "/data/openpilot/openpilot/selfdrive/controls/lib"

EDITS = {
  "longitudinal_planner.py": [
    ("""import math
import time""",
     """import math
import os
import time"""),
    ("""_TAKEOFF_TOTAL_MAX = [3.5, 3.5, 1.7, 1.7, 3.2]

def get_max_accel(v_ego):""",
     """_TAKEOFF_TOTAL_MAX = [3.5, 3.5, 1.7, 1.7, 3.2]

# /data/ops_patches/camry_small_step (read once at import): caps the cruise
# candidate's proportional target for driver set-speed gaps up to ~5 mph so a
# small tap stays below the THS engine-start threshold; linear blend back to
# stock across 2.2..3.6 m/s of gap, stock beyond. Flat vs gap (a floor, not a
# ramp-off) so it cannot collapse near set speed; symmetric up/down. Optional
# int contents = flat cap in cm/s^2 clamped 15..60 replacing the speed table;
# empty or unparseable = default table; absent = None = stock exactly.
SMALL_STEP_MARKER = "/data/ops_patches/camry_small_step"
_SMALL_STEP_FULL_MPS = 2.2    # full cap through ~5 mph of set-speed gap
_SMALL_STEP_STOCK_MPS = 3.6   # fully stock again by ~8 mph of gap
_SMALL_STEP_CAP_BP = [0., 10., 25., 40.]         # v_ego breakpoints, m/s
_SMALL_STEP_CAP_V = [0.35, 0.32, 0.27, 0.22]     # steady request, m/s^2


def _read_small_step_cap():
  if not os.path.isfile(SMALL_STEP_MARKER):
    return None
  try:
    with open(SMALL_STEP_MARKER) as f:
      raw = f.read().strip()
    if not raw:
      return list(_SMALL_STEP_CAP_V)
    return [min(60, max(15, int(raw))) / 100.0] * len(_SMALL_STEP_CAP_V)
  except (OSError, ValueError):
    return list(_SMALL_STEP_CAP_V)


SMALL_STEP_CAP = _read_small_step_cap()


def get_max_accel(v_ego):"""),
    ("""                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False):""",
     """                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     small_step: bool = False):"""),
    ("""  target_accel = np.clip(v_cruise - v_ego, cruise_min, max_accel)""",
     """  if small_step and SMALL_STEP_CAP is not None:
    # Cap only the proportional target's bounds, never beyond the stock ones.
    a_gentle = float(np.interp(v_ego, _SMALL_STEP_CAP_BP, SMALL_STEP_CAP))
    step_blend = float(np.clip((abs(v_cruise - v_ego) - _SMALL_STEP_FULL_MPS) /
                               (_SMALL_STEP_STOCK_MPS - _SMALL_STEP_FULL_MPS), 0., 1.))
    cap_lo = max((1.0 - step_blend) * -a_gentle + step_blend * cruise_min, cruise_min)
    cap_hi = min((1.0 - step_blend) * a_gentle + step_blend * max_accel, max_accel)
    cruise_min = min(cap_lo, cap_hi)
    max_accel = cap_hi
  target_accel = np.clip(v_cruise - v_ego, cruise_min, max_accel)"""),
    ("""    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, cruise_profile, slc_floor,
                                     base_brake_floor=global_floor, selected_acceleration_max=selected_acceleration, faster_takeoffs=faster_takeoffs)""",
     """    # Marker-gated gentle cap for the driver's own small set-speed gaps only;
    # any ceiling, stop, force-decel, coast plan or pedal keeps stock cruise.
    small_step = (SMALL_STEP_CAP is not None and ceiling.effective_mps is None and curve.effective_mps is None and
                  navigation_target is None and coast_plan is None and self.force_stop_plan.speed_ceiling_mps is None and
                  not self.force_stop_plan.forcing and not sm['controlsState'].forceDecel and
                  not sm['carState'].standstill and not sm['carState'].brakePressed and not sm['carState'].gasPressed)
    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, cruise_profile, slc_floor,
                                     base_brake_floor=global_floor, selected_acceleration_max=selected_acceleration, faster_takeoffs=faster_takeoffs,
                                     small_step=small_step)"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(PLANNER_DIR, name)
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
    print("camry small step already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(PLANNER_DIR, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
