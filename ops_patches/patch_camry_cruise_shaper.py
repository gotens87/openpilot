#!/usr/bin/env python3
# 2018 Camry Hybrid (THS eCVT), set-speed step response, v7.
#
# V7 = v6 (the grok4-signed-off shaper) rebased onto the upstream that
# gave get_cruise_accel the a_target_prev keyword and changed the
# cruise slew lower bound to
#   decel_start = a_cruise_prev if a_target_prev is None
#                 else min(a_cruise_prev, a_target_prev).
# camry_smooth_onset and camry_small_step are retired (smooth_onset's
# marker is off; small_step is superseded by this shaper), so v7
# applies to the UNPATCHED upstream longitudinal_planner.py -- no
# other ops patch runs before it. Rebase deltas, none of which touch
# the shaper's pulse law, gates, clamps or constants:
#
#   * the small_step parameter, the small_step= call argument and the
#     "small_step cleared while a pulse owns the candidate" line are
#     gone -- the cap patch no longer exists to interact with;
#   * the arm-time jerk cap reads the single upstream J_CRUISE_VALS
#     table; the retired smooth_onset split kept its down table at
#     these same stock values, so the cap is numerically identical;
#   * the post-slew hard clip still sits AFTER the stock slew line,
#     and that slew line keeps upstream's decel_start semantics
#     verbatim -- when output_a_target < a_cruise_prev the cruise
#     candidate's per-frame decel limit is now min(a_cruise_prev,
#     a_target_prev) - j*dt (upstream's change; it binds stock and
#     shaped candidates alike and the shaper neither weakens nor
#     strengthens it);
#   * import os is added here (it previously rode in with small_step).
#
# Unchanged from v6 -- the pulse law tuned to the owner's measured
# old-fork taps, the gates and every v4/v5/v6 safety fix:
#
#   * MUST-FIX 2 (corr double-count): the settle correction is 0 for
#     the whole cosine and runs only after it ends (the plan's done
#     flag rides in the shaper tuple). The pulse alone -- 0.80 of the
#     gap by construction -- can never carry ego through the set
#     speed; the leftover 20% creeps in under CORR after T (or is
#     handed to stock residue-free by the settle disarm), and the
#     0.3 m/s overshoot trip is a pure backstop.
#   * MUST-FIX 3 (clamp blocks closing): the monotone tap-frame clamp
#     is checked EVERY frame, not just at the tap: the plan is dropped
#     the same frame an up-tap floor at/above 0 sees ego past the set
#     speed (gap < 0), or a down-tap ceiling at/below 0 sees ego short
#     of it (gap > 0). And once the cosine has ended, a crossing
#     beyond the arm threshold (0.2 m/s) in EITHER direction hands
#     back to stock. The 0.3 m/s trip stays as the mid-pulse backstop.
#   * MUST-FIX 4 (jerk after duration clamp): after dur = min(8 s, ..)
#     the amplitude is re-capped so pi*A/dur <= J_CRUISE_VALS(v). The
#     area the re-cap removes goes to the settle phase, never to a
#     stock step.
#
# Retained from v5/v4, untouched by these fixes (grok3 must-fixes):
#
#   * Every button step of 0.2..2.5 m/s, either direction, re-plans a
#     fresh pulse from the CURRENT v_ego and a_cruise, zero slope at
#     both ends, peak jerk clamped to the cruise jerk table at v.
#   * The tap-frame command a_cruise is stored RAW (never max/min with
#     0) as a monotone clamp for the life of the plan: an up-tap floors
#     the command there, a down-tap ceilings it there. Enforced pre-
#     slew AND post-slew, every frame the plan lives, including the
#     settle phase after the cosine ends. After T the candidate is
#     feedforward 0 plus the CORR term on the LIVE gap (clamped like
#     everything else), never the raw stock gap clip. v8 (trace2
#     comfort fix): a POSITIVE up-tap floor -- a tap that landed on
#     the previous step's residue -- is enforced only capped at half
#     the pulse peak while the cosine runs and is released when it
#     ends, so it can hold a re-arm spin-up but never flatten the
#     pulse into a constant shove or pin the settle tail above the
#     corr term; negative floors (never deepen braking) and down-tap
#     ceilings (never raise the command) stay verbatim, and the raw
#     clamps still drive every disarm check.
#   * The clamp is released only when the gap is inside 0.2 m/s AND
#     |a_cruise| <= 0.15 (residue-free stock handoff), on a post-cosine
#     crossing past 0.2 m/s in either direction, on a >0.3 m/s
#     overshoot of set speed, on a clamp that would block a closing
#     command, or when any new step re-plans -- and a step larger than
#     2.5 m/s clears the plan the SAME frame, so resumes and
#     long-presses are stock.
#   * Accel-pulse amplitude is built from min(table, max_accel): the
#     arm-time cap mirrors get_cruise_accel's whole non-e2e max_accel
#     stack (speed table, selected/profile caps, turn limit
#     a_x_allowed, throttle-gate coast limit; e2e excluded by the
#     gate).
#   * Hard same-frame clip AFTER the shared stock slew, every cycle,
#     stock and shaped alike: bounds are cruise_min/max_accel
#     (max_accel already folds ACCEL_MAX, the turn limit, the coast
#     limit and every profile/selected cap for THIS cycle), and while
#     a plan owns the candidate the clip also carries the shaper peak
#     table, the decel floor max(cruise_min, -decel_cap) and the
#     monotone tap-frame clamps. Plan clamps yield to the safety caps
#     on conflict.
#
# Arms only for button steps of 0.2..2.5 m/s. Exclusions: any
# cruise/curve/nav ceiling, force-stop plan, an ACTING coast plan
# (force coast, pulse-glide coasting, brake floor, speed ceiling or
# force_decel -- the idle CoastPlan object always exists, so the
# object itself can never be the gate), forceDecel, standstill,
# driver pedal, experimental mode or reset disarms instantly.
# It shapes ONLY the cruise candidate: MPC/lead/e2e
# candidates, shouldStop, allowThrottle and all ceiling logic are
# untouched; min() candidate arbitration can never be weakened by a
# stronger cruise request; the shaped target still passes through the
# common stock slew (same dt; upstream's decel_start lower bound
# unchanged) AND the post-slew clip, with no early return in any
# branch.
#
# Optional marker contents: "gain_pct[,cap_cm[,decel_cm[,corr_centi]]]"
# (gain 40..200 = G0 in percent, default 40; cap 25..100 in cm/s^2,
# flat, replacing the speed table; decel 30..50 in cm/s^2 -- never
# stronger than stock cruise_min; corr 8..60 in 0.01/s, default 15).
# STRICT parse: fields must be 1..4 integers; a float, a blank, a
# trailing comma or a wrong field count returns None = stock exactly,
# with a warning logged. The resolved config is logged once at import.
# Empty file = defaults; absent marker = None = stock exactly.
#
# Supersedes camry_small_step (retired; delete that marker) and
# REPLACES v5/v6: if either was applied, revert
# longitudinal_planner.py to the unshaped upstream file first -- the
# anchors below match the unshaped text only and the patcher aborts
# safely on shaper text. The marker is import-time state: it takes
# effect when manager next starts the controls stack (each boot). No
# params are written. Deleting the marker reverts to stock after a
# controls restart; the in-car abort is the brake pedal.
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

# /data/ops_patches/camry_cruise_shaper (read once at import): on a
# driver set-speed step, closes the gap the way the owner's old
# (pre-7.0, Dom/StarPilot) fork did -- measured from his logs
# (old-fork-excerpt.md, 31 taps): a smooth accel pulse with a ~1-1.5 s
# ramp-on, peak at ~2.5-4 s, a ~6-9 s taper, then a slow creep into
# the new set speed (highway up-taps often had not arrived after 10 s;
# none overshot). Reproduced as a raised-cosine accel pulse: amplitude
# A = min(G(v)*|gap|, cap(v), max_accel) with G(v) = GAIN0*20/(v+10)
# (the old MPC's 1/(v+10) cruise-cost normalization; his logs imply a
# nearly flat G ~0.2-0.33 1/s across 50-120 km/h), duration
# T = 2*PULSE_FRAC*|gap|/A so the pulse area is PULSE_FRAC of the gap.
# v6 (grok4 must-fixes 2-4): the settle correction is 0 while the
# cosine runs -- the pulse area is already 0.80 of the gap, so a
# +1 km/h tap at 100 km/h (A 0.059, T 7.5 s, area 0.22 of the 0.28 m/s
# gap) can never carry ego through the new set speed; only after the
# cosine ends does CORR creep the leftover 20%, and the 0.3 m/s
# overshoot trip is a pure backstop, never the normal end of a
# one-tick. The monotone tap clamp is checked every frame and the plan
# is dropped the same frame it would block a closing command (up-tap
# floor at/above 0 while ego is past the set speed, down-tap ceiling
# at/below 0 while ego is short of it); once the cosine has ended, a
# crossing beyond the arm threshold hands back to stock in either
# direction. After the 8 s duration clamp the amplitude is re-capped
# so pi*A/dur <= J_CRUISE_VALS(v). Every in-range button step re-plans
# a fresh pulse from the current v_ego and a_cruise, and the tap-frame
# command becomes a monotone clamp for the life of the plan: an up-tap
# floors the command there (it can only hold or rise), a down-tap
# ceilings it there (it can only hold or fall), enforced pre- AND
# post-slew until the gap is closed and a_cruise has settled at ~0.
# Strict parse: any failure = None = stock; resolved config logged
# once at import.
CRUISE_SHAPER_MARKER = "/data/ops_patches/camry_cruise_shaper"
_SHAPER_GAIN0 = 0.4                            # speed-error gain scale, 1/s; G(v)=GAIN0*20/(v+10); his logs imply ~0.2-0.33 1/s over 50-120 km/h
_SHAPER_GAIN_MIN = 0.12
_SHAPER_GAIN_MAX = 0.9
_SHAPER_PULSE_FRAC = 0.80                      # fraction of the gap the cosine closes; the rest creeps in under CORR after the cosine ends (old fork: no overshoot, slow arrival)
_SHAPER_CAP_BP = [0., 10., 25., 40.]           # v_ego breakpoints, m/s
_SHAPER_CAP_V = [0.90, 0.80, 0.60, 0.45]       # peak pulse accel cap, m/s^2 (binds mainly below ~80 km/h and on the biggest steps; highway peaks are gain-bound ~0.45-0.55)
_SHAPER_DECEL = 0.5                            # shaped decel cap, m/s^2 (== stock A_CRUISE_MIN: never stronger than stock; measured down-taps -0.1..-0.54)
_SHAPER_CORR = 0.15                            # settle/correction gain on the LIVE gap, 1/s, POST-COSINE ONLY (old fork's creep tail ran ~0.03-0.06 m/s^2 on a ~0.3-0.6 m/s gap)
_SHAPER_CORR_MAX = 0.10                        # correction clamp, m/s^2 (stays well under the THS engine-start threshold)
_SHAPER_MIN_STEP = 0.2                         # arm threshold, m/s (1 km/h and 1 mph taps); also the post-cosine crossing/disarm band
_SHAPER_MAX_STEP = 2.5                         # largest shaped step, m/s; a bigger step clears the plan the same frame (resumes stay stock)
_SHAPER_MAX_DUR = 8.0                          # s; a longer pull under-closes and hands a bigger tail to the settle phase
_SHAPER_OVERSHOOT = 0.3                        # disarm to stock if ego passes set speed by this, m/s (mid-pulse backstop)
_SHAPER_BLEND = 0.4                            # s, blends current a_cruise into the pulse on (re)arm
_SHAPER_SETTLE_A = 0.15                        # disarm needs |a_cruise| at/below this with the gap closed, m/s^2


def _read_cruise_shaper():
  if not os.path.isfile(CRUISE_SHAPER_MARKER):
    return None
  try:
    with open(CRUISE_SHAPER_MARKER) as f:
      raw = f.read().strip()
    if not raw:
      cloudlog.info("camry_cruise_shaper: defaults gain=%.2f cap=%s decel=%.2f corr=%.2f",
                    _SHAPER_GAIN0, _SHAPER_CAP_V, _SHAPER_DECEL, _SHAPER_CORR)
      return (_SHAPER_GAIN0, list(_SHAPER_CAP_V), _SHAPER_DECEL, _SHAPER_CORR)
    parts = raw.split(",")
    if not 1 <= len(parts) <= 4:
      raise ValueError(f"want 1..4 fields, got {len(parts)}")
    vals = [int(p) for p in parts]  # strict: floats, blanks, trailing commas all raise
    gain = min(200, max(40, vals[0])) / 100.0
    cap = [min(100, max(25, vals[1])) / 100.0] * len(_SHAPER_CAP_BP) if len(vals) > 1 else list(_SHAPER_CAP_V)
    decel = min(50, max(30, vals[2])) / 100.0 if len(vals) > 2 else _SHAPER_DECEL
    corr = min(60, max(8, vals[3])) / 100.0 if len(vals) > 3 else _SHAPER_CORR
    cloudlog.info("camry_cruise_shaper: marker %r -> gain=%.2f cap=%s decel=%.2f corr=%.2f",
                  raw, gain, cap, decel, corr)
    return (gain, cap, decel, corr)
  except (OSError, ValueError) as e:
    cloudlog.warning("camry_cruise_shaper: marker parse failed (%s); stock cruise", e)
    return None


CRUISE_SHAPER = _read_cruise_shaper()


class _ShaperPlan:
  \"\"\"Raised-cosine accel pulse closing one set-speed step, old-fork style.

  a(t) = s * A/2 * (1 - cos(2*pi*t/T)) with A = min(G(v)*|gap|, cap,
  max_accel) and T = 2*PULSE_FRAC*|gap|/A, so the pulse area is
  PULSE_FRAC (0.80) of the gap -- the measured old-fork shape: peak at
  T/2 ~= PULSE_FRAC/G(v) (~3.8 s at 100 km/h), a ~7-8 s taper, then a
  slow no-overshoot creep on the remainder. Both ends have zero slope.
  A short blend carries the tap-frame a_cruise (a0) into the pulse so
  a mid-pull re-arm is continuous. Timing is monotonic-clock based,
  not frame dt, so a dropped frame cannot distort the profile.
  a_lo/a_hi are the monotone tap-frame clamps: an up-tap stores
  a_lo = a_cruise (the command can only hold or rise), a down-tap
  stores a_hi = a_cruise (it can only hold or fall). They are stored
  RAW for the life of the plan and drive every disarm check; v8
  relaxes only the ENFORCED positive up-tap floor (capped at half the
  pulse peak while the cosine runs, released when it ends) so a tap
  onto the previous step's residue cannot flatten the pulse into a
  constant push or pin the settle tail above CORR. get_cruise_accel
  enforces them pre- and post-slew for the whole life of the plan --
  including the settle phase after T, where the feedforward is 0 and
  the candidate is the clamped CORR term on the live gap (CORR is 0
  while the cosine runs: the pulse area is already 0.80 of the gap, so
  adding it then double-counts and can carry ego through the set
  speed) -- and they yield to the safety caps on conflict. update()
  drops the plan the same frame any clamp would block a closing
  command, on a post-cosine crossing past the arm threshold in either
  direction, or on a >0.3 m/s overshoot. There is no plan-integrated
  speed anywhere: the correction always tracks the live gap, so a live
  cap cutting the pulse leaves a residue the settle phase closes
  gently instead of an unbudgeted step handed to stock.
  \"\"\"
  __slots__ = ("t0_ns", "a0", "A", "T", "w", "s", "a_lo", "a_hi")

  def __init__(self, t0_ns, a0, amp, dur, s, a_lo=-math.inf, a_hi=math.inf):
    self.t0_ns = t0_ns
    self.a0 = a0
    self.A = amp
    self.T = dur
    self.w = 2.0 * math.pi / max(dur, 0.1)
    self.s = s
    self.a_lo = a_lo
    self.a_hi = a_hi

  def eval(self, now_ns):
    t = max(0.0, (now_ns - self.t0_ns) / 1e9)
    if t >= self.T:
      return 0.0, True
    pulse = 0.5 * self.A * (1.0 - math.cos(self.w * t))
    blend = min(t / _SHAPER_BLEND, 1.0)
    return (1.0 - blend) * self.a0 + blend * self.s * pulse, False


def get_max_accel(v_ego):"""),
    ("""                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     *, a_target_prev: float | None = None):""",
     """                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     *, a_target_prev: float | None = None, shaper: tuple[float, float, float, bool] | None = None):"""),
    ("""  target_accel = np.clip(v_cruise - v_ego, cruise_min, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)""",
     """  if shaper is not None and CRUISE_SHAPER is not None and not e2e:
    # Raised-cosine pulse following, not a P gain: plan feedforward, with
    # the settle correction added ONLY after the cosine has ended
    # (shaper[3]). While the cosine runs, the pulse area is already
    # PULSE_FRAC (0.80) of the gap; adding corr on top double-counts it
    # -- in v5 a +1 km/h tap at 100 km/h crossed the new set speed near
    # 3.5 s and tripped the overshoot backstop as the NORMAL end of a
    # one-tick (grok4 must-fix 2). After T the feedforward is 0 and corr
    # creeps the leftover 20% on the LIVE gap (never a plan-integrated
    # speed -- if a live cap cuts the pulse, the correction still tracks
    # reality; grok3 must-fix 4). Hard clamps bind THIS frame, every
    # cycle: max_accel already folds in ACCEL_MAX, the turn limit
    # (a_x_allowed), the throttle-gate coast limit and every
    # profile/selected cap; the cap table bounds the pulse peak; the
    # decel cap keeps shaped braking at or below the stock cruise floor;
    # shaper[1]/shaper[2] are the monotone tap-frame clamps (up-tap
    # floor / down-tap ceiling; a positive floor is capped at half the
    # pulse peak while the cosine runs and released after it ends --
    # v8) and bind the correction term too; they yield to the safety
    # caps on conflict. NO early return: the common
    # stock slew below (same dt; upstream's decel_start lower bound is
    # unchanged) and the post-slew clip still run every cycle.
    cap_hi = min(max_accel, float(np.interp(v_ego, _SHAPER_CAP_BP, CRUISE_SHAPER[1])))
    cap_lo = max(cruise_min, -CRUISE_SHAPER[2])
    plan_lo = max(cap_lo, shaper[1])
    plan_hi = min(cap_hi, shaper[2])
    if plan_lo <= plan_hi:
      cap_lo, cap_hi = plan_lo, plan_hi
    corr = 0.0
    if shaper[3]:
      corr = float(np.clip(CRUISE_SHAPER[3] * (v_cruise - v_ego), -_SHAPER_CORR_MAX, _SHAPER_CORR_MAX))
    target_accel = float(np.clip(shaper[0] + corr, cap_lo, cap_hi))
  else:
    target_accel = np.clip(v_cruise - v_ego, cruise_min, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)"""),
    ("""  target_accel = float(np.clip(target_accel, decel_start - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel""",
     """  target_accel = float(np.clip(target_accel, decel_start - j_cruise * dt, a_cruise_prev + j_cruise * dt))
  # Same-frame hard bounds AFTER the slew (grok3 must-fixes 1 and 2),
  # every cycle, stock and shaped alike: max_accel has already folded in
  # ACCEL_MAX, the turn limit a_x_allowed, the throttle-gate coast limit
  # and every profile/selected cap for THIS cycle, and cruise_min holds
  # the brake floors (ACCEL_MIN, the global/coast floors, the profile
  # brake magnitude). Without this clip a cap that drops under a live
  # pull is walked off at the slew rate for up to ~1 s. While a shaped
  # plan owns the candidate the clip also carries the shaper peak table,
  # the decel floor max(cruise_min, -decel_cap) and the monotone
  # tap-frame clamps -- so a down-tap ceiling / up-tap floor binds after
  # the slew too, on every frame the plan lives, including the frame the
  # cosine ends (a positive up-tap floor is capped at half the pulse
  # peak and released once the cosine ends -- v8; the raw clamps still
  # drive the disarm checks). a_x_allowed and ACCEL_MAX are not named
  # here: a_x_allowed is only bound in the non-e2e branch above, and
  # both are already inside max_accel. Plan clamps yield to the safety caps on
  # conflict. The clip is a no-op whenever the previous command was
  # already inside the current caps.
  post_lo, post_hi = cruise_min, max_accel
  if shaper is not None and CRUISE_SHAPER is not None and not e2e:
    shaper_lo = max(cruise_min, -CRUISE_SHAPER[2], shaper[1])
    shaper_hi = min(max_accel, float(np.interp(v_ego, _SHAPER_CAP_BP, CRUISE_SHAPER[1])), shaper[2])
    if shaper_lo <= shaper_hi:
      post_lo, post_hi = shaper_lo, shaper_hi
  target_accel = float(np.clip(target_accel, post_lo, post_hi))

  return target_accel"""),
    ("""    self.lead_approach = LeadApproach(dt, CP.longitudinalActuatorDelay)""",
     """    self.lead_approach = LeadApproach(dt, CP.longitudinalActuatorDelay)
    self.cruise_shaper_plan = None
    self.cruise_shaper_v_cruise = None"""),
    ("""    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target""",
     """    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      self.cruise_shaper_plan = None  # grok3 must-fix 3: the plan dies with the state reset, not one gate check later"""),
    ("""    if coast_plan is not None and coast_plan.brake_floor is not None:
      global_floor = coast_plan.brake_floor
      cruise_profile = None
      slc_floor = None
    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,""",
     """    if coast_plan is not None and coast_plan.brake_floor is not None:
      global_floor = coast_plan.brake_floor
      cruise_profile = None
      slc_floor = None
    # Marker-gated shaped set-speed response: the gap is closed the
    # way the owner's old fork measurably did (old-fork-excerpt.md) --
    # a raised-cosine pulse peaking at ~PULSE_FRAC/G(v) (~3.8 s at
    # 100 km/h, measured 2.5-4 s), area = PULSE_FRAC of the gap, then
    # a slow no-overshoot creep on the remainder. (grok4 must-fixes:)
    #   * the settle correction is 0 while the cosine runs, so the
    #     pulse alone -- 80% of the gap by construction -- can never
    #     carry ego through the new set speed; corr creeps only the
    #     leftover 20% after T, and the 0.3 m/s overshoot trip is a
    #     pure backstop, never the normal end of a one-tick.
    #   * the monotone tap clamp is checked EVERY frame: an up-tap
    #     floor at/above 0 while ego is past the set speed (gap < 0),
    #     or a down-tap ceiling at/below 0 while ego is short of it
    #     (gap > 0), would block the closing command, so the plan is
    #     dropped that same frame; and once the cosine has ended, a
    #     crossing beyond the arm threshold (0.2 m/s) in either
    #     direction hands back to stock -- the 0.2..0.3 m/s band the
    #     v5 settle/overshoot pair left armed is covered both ways.
    #   * after the 8 s duration clamp the amplitude is re-capped so
    #     peak cosine jerk pi*A/dur stays at/under the cruise jerk
    #     table at this speed.
    # EVERY in-range button step, either direction, re-plans a fresh
    # pulse from the current v_ego and a_cruise (grok3 must-fix 2):
    # the tap-frame command is stored RAW as a monotone clamp --
    # up-tap: floor (the command can only hold or rise); down-tap:
    # ceiling (it can only hold or fall) -- enforced pre- and
    # post-slew for the whole life of the plan, including the settle
    # phase after the cosine ends, so a down-tap can never raise the
    # command and an up-tap can never deepen braking, and there is no
    # frame where the clamp dies early. (v8 comfort fix, trace2: the
    # ENFORCED positive up-tap floor is capped at half the pulse peak
    # while the cosine runs and released when it ends -- the raw clamp
    # stays on the plan and drives every disarm check -- so a small
    # tap onto the previous step's residue swells gently instead of
    # holding the residue level for seconds and cutting at the
    # crossing.) A step larger than 2.5 m/s
    # clears the plan the same frame (resumes stay stock). Exclusions:
    # any cruise/curve/nav ceiling, force-stop plan, an ACTING coast
    # plan (force coast, pulse-glide coasting, brake floor, speed
    # ceiling or force_decel), forceDecel, standstill, driver pedal,
    # experimental mode or reset disarms instantly. Disarms only when
    # the gap is inside
    # 0.2 m/s AND a_cruise is already near 0 (residue-free stock
    # handoff), or instantly on a clamp that blocks closing, a
    # post-cosine crossing past 0.2 m/s, a >0.3 m/s overshoot of set
    # speed, or any gate.
    # CoastOwner.sample ALWAYS returns a CoastPlan instance (idle:
    # force_decel=False, brake_floor=None, speed_ceiling_mps=None,
    # pulse_coasting=False) and plannerd always passes a CoastRuntime,
    # so "coast_plan is None" was never true and the shaper could
    # never arm. Gate on coast ACTING: force coast (force_decel and
    # its -0.6 floor), pulse-glide coasting (pulse_coasting, -0.03
    # floor, speed ceiling), or any brake floor / speed ceiling.
    coast_acting = (coast_plan is not None and
                    (coast_plan.force_decel or coast_plan.pulse_coasting or
                     coast_plan.brake_floor is not None or coast_plan.speed_ceiling_mps is not None))
    shaper_gate = (CRUISE_SHAPER is not None and not reset_state and
                   not sm['selfdriveState'].experimentalMode and
                   ceiling.effective_mps is None and curve.effective_mps is None and
                   navigation_target is None and not coast_acting and
                   self.force_stop_plan.speed_ceiling_mps is None and
                   not self.force_stop_plan.forcing and not sm['controlsState'].forceDecel and
                   not sm['carState'].standstill and not sm['carState'].brakePressed and not sm['carState'].gasPressed)
    shaper_cmd = None
    if not shaper_gate:
      self.cruise_shaper_plan = None
      self.cruise_shaper_v_cruise = v_cruise
    else:
      prev_set = self.cruise_shaper_v_cruise
      self.cruise_shaper_v_cruise = v_cruise
      step = (v_cruise - prev_set) if prev_set is not None else 0.0
      if abs(step) > _SHAPER_MAX_STEP:
        # Resume / long-press: stock, and any live plan dies this same
        # frame (grok3 must-fix 2).
        self.cruise_shaper_plan = None
      elif abs(step) >= _SHAPER_MIN_STEP:
        gap = v_cruise - v_ego
        if step > 0.0:
          plan_lo, plan_hi = self.a_cruise, math.inf
        else:
          plan_lo, plan_hi = -math.inf, self.a_cruise
        gain = min(_SHAPER_GAIN_MAX, max(_SHAPER_GAIN_MIN, CRUISE_SHAPER[0] * 20.0 / (v_ego + 10.0)))
        if gap > 0.0:
          # Accel pulse: amplitude from the measured-fit speed-scaled
          # gain, the cap table, and THIS cycle's max_accel stack
          # (grok3 must-fix 4) -- selected/profile caps, the turn limit
          # and the throttle-gate coast limit, mirroring
          # get_cruise_accel's non-e2e branch (e2e is excluded by the
          # gate). Live caps that drop further still bind post-slew
          # every frame.
          arm_max = get_max_accel(v_ego)
          if selected_acceleration is not None:
            arm_max = min(arm_max, selected_acceleration, ACCEL_MAX)
          if cruise_profile is not None:
            arm_max = min(arm_max, cruise_profile.acceleration_max, ACCEL_MAX)
          a_total_max = (np.interp(v_ego, _TAKEOFF_TOTAL_BP, _TAKEOFF_TOTAL_MAX) if faster_takeoffs else
                         np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V))
          a_y = v_ego ** 2 * steer_angle_without_offset * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)
          arm_max = min(arm_max, math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.)))
          if not self.allow_throttle:
            arm_max = min(arm_max, float(np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
                                                   [arm_max, max(accel_coast, ACCEL_MIN)])))
          amp = min(gain * gap, float(np.interp(v_cruise, _SHAPER_CAP_BP, CRUISE_SHAPER[1])), arm_max)
          if step < 0.0:
            # Disagreeing down-tap while still below set: the pulse is
            # sized to the clamp it lives under -- it may not peak above
            # the tap-frame command (grok3 must-fix 2).
            amp = min(amp, max(self.a_cruise, 0.0))
        else:
          # Decel pulse: never stronger than the stock cruise floor.
          amp = min(gain * abs(gap), CRUISE_SHAPER[2])
          if step > 0.0:
            # Disagreeing up-tap while still above set: sized to the
            # tap-frame floor.
            amp = min(amp, max(-self.a_cruise, 0.0))
        # Peak cosine jerk is pi*A/T = pi*A^2/(2*PULSE_FRAC*|gap|); keep
        # it at or under the cruise jerk table at this speed (grok3
        # must-fix 4). The retired smooth_onset down table held these
        # same stock values, so the cap is numerically unchanged.
        # Shaped peak jerk is ~0.05 m/s^3 (+2 km/h at 100 km/h) to
        # ~0.20 (+8) -- the measured old-fork rise slope was ~0.2.
        j_down = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS))
        amp = max(0.0, min(amp, math.sqrt(2.0 * _SHAPER_PULSE_FRAC * abs(gap) * j_down / math.pi)))
        dur = min(_SHAPER_MAX_DUR, 2.0 * _SHAPER_PULSE_FRAC * abs(gap) / max(amp, 0.05))
        # grok4 must-fix 4: the duration clamp shortens T without
        # changing A, which would steepen peak jerk past the arm-time
        # limit -- re-cap A against the CLAMPED duration so
        # pi*A/dur <= J_CRUISE_VALS(v) still holds. Never re-raise A:
        # lifting the peak past the tap-frame command is the surge the
        # clamp exists to prevent. The pulse closes PULSE_FRAC of the
        # gap by design; the tail (plus any MAX_DUR shortfall and this
        # re-cap's area loss) is closed by the settle phase at CORR
        # gain under the same clamps, never by a stock step.
        amp = min(amp, j_down * dur / math.pi)
        self.cruise_shaper_plan = _ShaperPlan(sample_now_ns, self.a_cruise, amp, dur,
                                              1.0 if gap >= 0.0 else -1.0, plan_lo, plan_hi)
      if self.cruise_shaper_plan is not None:
        plan = self.cruise_shaper_plan
        a_ff, done = plan.eval(sample_now_ns)
        gap = v_cruise - v_ego
        # grok4 must-fix 3, checked EVERY frame (a fresh arm included):
        # the monotone tap clamp must never block a closing command. An
        # up-tap floor at/above 0 while ego is past the set speed
        # forbids the brake side; a down-tap ceiling at/below 0 while
        # ego is short of it forbids the throttle side. Drop the plan
        # the same frame -- stock owns the crossing. (At/above and
        # at/below, not strict: a floor of exactly 0 still forbids a
        # closing brake command.)
        clamp_blocks_closing = (plan.a_lo >= 0.0 and gap < 0.0) or (plan.a_hi <= 0.0 and gap > 0.0)
        # Overshoot backstop: 0.3 m/s past the set speed against the
        # pulse direction, at any time (a downhill surge mid-pulse).
        overshot = gap * plan.s < -_SHAPER_OVERSHOOT
        # Clean handoff: cosine done, gap inside the arm threshold and
        # a_cruise already near 0, so stock inherits no residue.
        settled = done and abs(gap) <= _SHAPER_MIN_STEP and abs(self.a_cruise) <= _SHAPER_SETTLE_A
        # grok4 must-fix 3: once the cosine has ended, a crossing beyond
        # the arm threshold hands back to stock in BOTH directions --
        # covers the 0.2..0.3 m/s band the v5 settle/overshoot pair
        # left armed with the clamp blocking the correction. The 0.3
        # trip above stays the mid-pulse backstop.
        crossed = done and gap * plan.s < -_SHAPER_MIN_STEP
        if clamp_blocks_closing or overshot or settled or crossed:
          self.cruise_shaper_plan = None
        else:
          # v8 comfort fix (trace2 hold/cut): the plan keeps the RAW
          # tap-frame clamps and every disarm check above still uses
          # them; only the ENFORCED up-tap floor is relaxed, and only
          # when it is positive (a tap that landed on the previous
          # step's residue). While the cosine runs it is capped at
          # half the pulse peak: enough to hold a re-arm spin-up (no
          # dip), never enough to flatten the whole pulse into a
          # constant shove. Once the cosine ends it is released, so
          # the settle phase creeps at the corr term instead of
          # pinning at the residue (in v7 a +1 km/h tap onto a 0.13
          # residue held a flat 0.13 for ~4 s then cut to ~0 at the
          # crossing, and the +8 settle tail pinned at 0.13 above the
          # 0.10 corr clamp). The command may relax below the
          # tap-frame value only within positive accel -- never below
          # half the freshly requested pulse, never into braking:
          # negative floors stay verbatim (an up-tap can never deepen
          # braking) and down-tap ceilings are untouched (a down-tap
          # can never raise the command).
          if plan.a_lo >= 0.0:
            eff_lo = -math.inf if done else min(plan.a_lo, 0.5 * plan.A)
          else:
            eff_lo = plan.a_lo
          shaper_cmd = (a_ff, eff_lo, plan.a_hi, done)
    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,"""),
    ("""                                     base_brake_floor=global_floor, selected_acceleration_max=selected_acceleration, faster_takeoffs=faster_takeoffs,
                                     a_target_prev=self.output_a_target)""",
     """                                     base_brake_floor=global_floor, selected_acceleration_max=selected_acceleration, faster_takeoffs=faster_takeoffs,
                                     a_target_prev=self.output_a_target, shaper=shaper_cmd)"""),
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
        if "_ShaperPlan" in content or "CRUISE_SHAPER" in content:
          print("  hint: shaper text detected; revert longitudinal_planner.py to the unshaped file before applying v7")
        return 1
      todo.append((old, new))
    pending[name] = todo

  if not any(pending.values()):
    print("camry cruise shaper already applied")
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
