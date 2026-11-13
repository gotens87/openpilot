#!/usr/bin/env python3
# Smooth Set-Speed Taps on Hybrid Camry
#
# 2018 Camry Hybrid (THS eCVT). A set-speed button tap is shaped instead
# of being handed to stock cruise as a step. Always on. The limits are
# constants in the patched planner.
#
# The shape is a jerk-limited trapezoid in acceleration, an S-curve in
# speed. Rise jerk and release jerk scale with the gap when the tap is
# armed: full limits (1.0 and 0.7 m/s^3) at 4 km/h and up, linear below
# that. A 1 km/h tap stays a short triangle, about 2.3 s, peaking near
# 0.24 m/s^2. At low speed, taps of 2 and 3 km/h hold a 0.35 m/s^2
# ceiling (about 2.4 s and 3.0 s). From 90 to 100.8 km/h the small-step
# law crossfades to a highway ceiling that rises with the tap
# (0.18 m/s^2 at 2 km/h, 0.43 at 4 km/h). Both laws blend into the
# speed table by 5 km/h, and taps of 5 km/h and up keep the full jerk
# limits. The accel ceiling follows the speed table, from 1.0 m/s^2 at
# a stop down to 0.6 m/s^2 at high speed. Shaped braking is never
# stronger than stock.
#
# Each frame the plan rises from the command captured on the tap, holds
# at the ceiling, then releases off the live gap. Release starts when
# the remaining gap is at most a squared over twice the release jerk,
# so the command hits zero as the car reaches the set speed. It
# integrates the live cruise acceleration, so a dropped frame, a live
# cap, the slew, or actuator lag only stretches the hold. The landing
# stays intact. There is no gain table, no pulse budget, and no
# correction tail. Duration is only a watchdog: 1.6 times the trapezoid
# estimate, plus 2 s. A new tap re-plans from the current speed and
# command.
#
# Shaping runs only while the car is within 8 km/h of the set speed.
# Farther out, stock cruise runs. A button step of at least 0.2 m/s
# re-plans. The raw tap-frame command is checked every frame; if that
# clamp would block a command that is still closing the gap, the plan
# drops on that frame. Crossing the set speed by more than 0.2 m/s in
# either direction, at any phase, hands back to stock. Handoff is clean
# once the gap is within 0.2 m/s and cruise acceleration is within
# 0.15 m/s^2. The ceiling at arm time follows the same non-experimental
# max-accel stack as get_cruise_accel, including the turn limit and the
# coast limit. Plan jerks are capped to the stock cruise jerk table, so
# the stock slew does not fight the plan. A hard clip after the slew
# runs every cycle, stock and shaped alike. Only the cruise candidate
# is shaped. Nothing returns early, and decel-start handling is
# unchanged.
#
# Shaping stays off for a cruise, curve, or nav ceiling, a force-stop
# plan, a coast plan that is actually braking or capping speed,
# force-decel, standstill, the driver brake or throttle, experimental
# mode, or a state reset.
#
# Applying again is a no-op when this text is already present. If older
# shaper text is in the planner, the patcher aborts; restore
# longitudinal_planner.py to the unshaped file first.
import os
import sys

PLANNER_DIR = "/data/openpilot/openpilot/selfdrive/controls/lib"

EDITS = {
  "longitudinal_planner.py": [
    ("""_TAKEOFF_TOTAL_MAX = [3.5, 3.5, 3.5, 1.7, 3.2]

def get_max_accel(v_ego, faster_takeoffs=False):""",
     """_TAKEOFF_TOTAL_MAX = [3.5, 3.5, 3.5, 1.7, 3.2]

# Always-on jerk-limited trapezoid in accel (S-curve in speed): RISE at
# j_up, HOLD at ceiling A, RELEASE at -j_dn off the LIVE gap (release
# when remaining <= a^2/(2*j_dn): command reaches 0 exactly as ego
# reaches set speed). Jerk scales with the arm-time gap: full j_up/j_dn
# at +4 km/h and up, linear below; +1 stays a ~2.3 s triangle (peak
# 0.24), +2/+3 hold the 0.35 small-step ceiling (grok must-fix) up to
# 90 km/h; past 100.8 km/h the small ceiling rises with the step
# (0.18 @ +2 -> 0.43 @ +4) so +2..+5 arrivals stay monotone; both laws
# blend to the table by +5, so +5/+9 keep the arrivals. Integrates the
# live a_cruise -> continuous by construction; no gain table, no pulse
# budget, no corr tail, no fixed duration to re-cap. Limits are fixed
# constants.
_SHAPER_J_UP = 1.0      # full-step rise jerk, m/s^3
_SHAPER_J_DN = 0.7      # full-step release jerk, m/s^3
_SHAPER_J_STEP = 1.11   # gap (m/s) for full jerk = +4 km/h; smaller steps scale j linearly
_SHAPER_SMALL_CAP = 0.35  # small-step accel ceiling, m/s^2 (low speed): triangle crests 0.48/0.72 push rpm past the 1500-1600 band
_SHAPER_SMALL_V_BP = [25., 28.]  # v_ego m/s (90 -> 100.8 km/h): crossfade low-speed -> highway small-step law
_SHAPER_SMALL_HI_BP = [2. / 3.6, 4. / 3.6, 5. / 3.6]  # gap (m/s): highway ceiling rises +2 -> +4, blends to the cap table by +5
_SHAPER_SMALL_HI_V = [0.18, 0.43]  # highway ceiling @ +2 / +4 km/h: +2 ~3.5 s profile (~4 s with lag); arrivals monotone +2..+5
_SHAPER_SMALL_BP = [3. / 3.6, 5. / 3.6]  # gap (m/s): flat small ceiling through +3 km/h, linear blend to the cap table by +5 (low-speed law, unchanged)
_SHAPER_CAP_BP = [0., 10., 25., 40.]
_SHAPER_CAP_V = [1.00, 0.90, 0.65, 0.60]  # ceiling m/s^2; 0.65 @ 90 km/h ~ 2000-2200 rpm steady
_SHAPER_DECEL = 0.5     # decel ceiling == stock A_CRUISE_MIN: never stronger than stock
_SHAPER_MIN_STEP = 0.2  # arm threshold, m/s; also the any-phase crossing/disarm band
_SHAPER_MAX_GAP = 8 / 3.6  # remaining |v_cruise-v_ego|; outside this, stock
_SHAPER_SETTLE_A = 0.15 # disarm needs |a_cruise| at/below this with the gap closed, m/s^2
_SHAPER_WD_MUL = 1.6    # watchdog: mul * trapezoid estimate + pad
_SHAPER_WD_PAD = 2.0


CRUISE_SHAPER = (_SHAPER_J_UP, _SHAPER_J_DN, list(_SHAPER_CAP_V), _SHAPER_DECEL)


class _ShaperPlan:
  \"\"\"Jerk-limited trapezoid for one set-speed step; STATELESS law:
  step() integrates the live a_cruise and releases off the live gap,
  so a dropped frame, a live cap, the slew or actuator lag only
  stretch the hold, never corrupt the landing. a_lo/a_hi are the RAW
  monotone tap-frame clamps: they drive every disarm check in
  update(). The ENFORCED side only bars the wrong sign (up-plan never
  brakes below 0, down-plan never throttles above 0): the law is
  monotone in the closing direction from the tap-frame command, so
  nothing can pin the pull or the tail. t_wd is a watchdog deadline,
  not the profile.\"\"\"
  __slots__ = ("t0_ns", "last_ns", "s", "A", "j_up", "j_dn", "a_lo", "a_hi", "t_wd")

  def __init__(self, t0_ns, s, A, j_up, j_dn, a_lo, a_hi, t_wd):
    self.t0_ns = t0_ns
    self.last_ns = t0_ns
    self.s = s
    self.A = A
    self.j_up = j_up
    self.j_dn = j_dn
    self.a_lo = a_lo
    self.a_hi = a_hi
    self.t_wd = t_wd

  def step(self, now_ns, a_cruise, gap):
    dt = min(0.1, max(0.0, (now_ns - self.last_ns) / 1e9))  # monotonic clock, not frame dt
    self.last_ns = now_ns
    g = gap * self.s        # remaining closing distance, m/s
    a = a_cruise * self.s   # command in the closing direction
    if g <= max(a, 0.0) ** 2 / (2.0 * self.j_dn):
      a -= self.j_dn * dt   # release: reaches 0 as g reaches 0 (also bleeds a small crossing)
    elif a > self.A:
      a = max(a - self.j_dn * dt, self.A)  # residue above ceiling: ease down
    elif a < self.A:
      a = min(a + self.j_up * dt, self.A)  # rise (first frame after arm: dt=0, a_ff=a_cruise, no jump)
    return self.s * a


def get_max_accel(v_ego, faster_takeoffs=False):"""),
    ("""                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     *, a_target_prev: float | None = None):""",
     """                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     *, a_target_prev: float | None = None, shaper: tuple[float, float, float] | None = None):"""),
    ("""  speed_error = v_cruise - v_ego
  if not e2e and not faster_takeoffs and v_ego >= MIN_ALLOW_THROTTLE_SPEED and speed_error > 0.:
    speed_error /= CRUISE_ACCEL_RESPONSE_TIME
  target_accel = np.clip(speed_error, cruise_min, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)""",
     """  speed_error = v_cruise - v_ego
  if not e2e and not faster_takeoffs and v_ego >= MIN_ALLOW_THROTTLE_SPEED and speed_error > 0.:
    speed_error /= CRUISE_ACCEL_RESPONSE_TIME
  if shaper is not None and not e2e:
    # The plan's feedforward IS the target (no P gain, no corr, no
    # blend). Hard clamps bind THIS frame, every cycle: max_accel folds
    # ACCEL_MAX, the turn limit, the coast limit and every
    # profile/selected cap; the ceiling table bounds the pull; the
    # decel cap keeps shaped braking at/below the stock floor;
    # shaper[1]/shaper[2] are the sign-side tap clamps (the raw clamps
    # live on the plan and drive the disarms); plan clamps yield to
    # the safety caps on conflict. NO early return: the stock slew
    # below (same dt; decel_start unchanged) and the post-slew clip
    # still run every cycle.
    cap_hi = min(max_accel, float(np.interp(v_ego, _SHAPER_CAP_BP, CRUISE_SHAPER[2])))
    cap_lo = max(cruise_min, -CRUISE_SHAPER[3])
    plan_lo = max(cap_lo, shaper[1])
    plan_hi = min(cap_hi, shaper[2])
    if plan_lo <= plan_hi:
      cap_lo, cap_hi = plan_lo, plan_hi
    target_accel = float(np.clip(shaper[0], cap_lo, cap_hi))
  else:
    target_accel = np.clip(speed_error, cruise_min, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)"""),
    ("""  target_accel = float(np.clip(target_accel, decel_start - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel""",
     """  target_accel = float(np.clip(target_accel, decel_start - j_cruise * dt, a_cruise_prev + j_cruise * dt))
  # Same-frame hard bounds AFTER the slew, every cycle, stock and
  # shaped alike: max_accel has folded in ACCEL_MAX, the turn limit,
  # the coast limit and every profile/selected cap for THIS cycle, and
  # cruise_min holds the brake floors. Without this clip a cap that
  # drops under a live pull is walked off at the slew rate for up to
  # ~1 s. While a plan owns the candidate the clip also carries the
  # ceiling table, the decel floor max(cruise_min, -decel_cap) and the
  # sign-side tap clamps. Plan clamps yield to the safety caps on
  # conflict; the clip is a no-op whenever the previous command was
  # already inside the current caps.
  post_lo, post_hi = cruise_min, max_accel
  if shaper is not None and not e2e:
    shaper_lo = max(cruise_min, -CRUISE_SHAPER[3], shaper[1])
    shaper_hi = min(max_accel, float(np.interp(v_ego, _SHAPER_CAP_BP, CRUISE_SHAPER[2])), shaper[2])
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
    ("""    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,""",
     """    # Shaped set-speed response, always on: jerk-limited trapezoid
    # (S-curve in speed), jerk scaled with the arm-time gap (full
    # j_up/j_dn at +4 km/h and up): rise from the live command, hold
    # at the ceiling, release at -j_dn off the LIVE gap (remaining <=
    # a^2/(2*j_dn)), so the command reaches 0 exactly as ego reaches
    # the set speed; tach rises, holds, eases (no step anywhere).
    # EVERY in-range button step, either direction, re-plans from the
    # current v_ego and a_cruise; the tap-frame command is stored RAW
    # as the monotone clamp (up-tap floor / down-tap ceiling) and
    # drives every disarm check, checked EVERY frame: a clamp that
    # would block a closing command drops the plan the same frame; a
    # crossing past 0.2 m/s in either direction at ANY phase hands
    # back to stock; a watchdog (1.6x estimate + 2 s) drops a
    # cap-starved plan; the settle handoff is residue-free (gap <=
    # 0.2 AND |a_cruise| <= 0.15). Remaining |set-ego| > 8 km/h
    # clears the plan the same frame (stock). Exclusions: any
    # cruise/curve/nav ceiling, force-stop plan, an ACTING coast plan,
    # forceDecel, standstill, driver pedal, experimental mode or
    # reset disarms instantly.
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
    shaper_gate = (not reset_state and
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
      gap = v_cruise - v_ego
      if abs(gap) > _SHAPER_MAX_GAP:
        # More than 8 km/h off set: stock, and any live plan dies.
        self.cruise_shaper_plan = None
      elif abs(step) >= _SHAPER_MIN_STEP:
        if step > 0.0:
          plan_lo, plan_hi = self.a_cruise, math.inf
        else:
          plan_lo, plan_hi = -math.inf, self.a_cruise
        # Jerk scales with the gap: full j_up/j_dn at +4 km/h and up,
        # linear below; +1 stays a ~2.3 s soft triangle (peak 0.24)
        # and +2/+3 are held to the small-step ceiling below. Capped
        # to the cruise jerk table so the stock slew is a no-op on
        # the shaped candidate.
        j_tbl = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS))
        j_scale = min(1.0, abs(gap) / _SHAPER_J_STEP)
        j_up = max(0.05, min(CRUISE_SHAPER[0] * j_scale, j_tbl))
        j_dn = max(0.05, min(CRUISE_SHAPER[1] * j_scale, j_tbl))
        if gap > 0.0:
          # Accel ceiling: the cap table and THIS cycle's max_accel
          # stack -- selected/profile caps, the turn limit and the
          # throttle-gate coast limit, mirroring get_cruise_accel's
          # non-e2e branch (e2e is excluded by the gate). Live caps
          # that drop further still bind post-slew every frame.
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
          cap_tbl = float(np.interp(v_cruise, _SHAPER_CAP_BP, CRUISE_SHAPER[2]))
          # Small-step ceiling (grok must-fix), two laws crossfaded
          # 90 -> 100.8 km/h. Low speed (unchanged): flat 0.35 through
          # +3 km/h, blend to the cap table by +5. Highway: the
          # ceiling RISES with the step (0.18 @ +2 -> 0.43 @ +4, blend
          # to the table by +5) -- a flat ceiling cannot stretch +2 to
          # ~4 s and keep +3 under ~4 s (grok v11 NO: flat 0.18 made
          # +3 4.9 s, slower than +4). Both laws end at the cap table
          # by +5 (+5 and up unchanged). Continuous in gap and speed.
          small_lo = float(np.interp(gap, _SHAPER_SMALL_BP, [_SHAPER_SMALL_CAP, cap_tbl]))
          small_hi = float(np.interp(gap, _SHAPER_SMALL_HI_BP,
                                     [_SHAPER_SMALL_HI_V[0], _SHAPER_SMALL_HI_V[1], cap_tbl]))
          small_cap = float(np.interp(v_ego, _SHAPER_SMALL_V_BP, [small_lo, small_hi]))
          cap_a = min(cap_tbl, arm_max, small_cap)
          if step < 0.0:
            # Disagreeing down-tap while still below set: sized to the
            # ceiling it lives under.
            cap_a = min(cap_a, max(self.a_cruise, 0.0))
        else:
          # Decel ceiling: never stronger than the stock cruise floor.
          cap_a = CRUISE_SHAPER[3]
          if step > 0.0:
            # Disagreeing up-tap while still above set: sized to the
            # tap-frame floor.
            cap_a = min(cap_a, max(-self.a_cruise, 0.0))
        cap_a = max(cap_a, 0.0)
        # Watchdog only: trapezoid estimate (triangular steps
        # overestimate it -- the safe direction for a watchdog).
        dur = abs(gap) / max(cap_a, 0.05) + cap_a / (2.0 * j_up) + cap_a / (2.0 * j_dn)
        self.cruise_shaper_plan = _ShaperPlan(sample_now_ns, 1.0 if gap >= 0.0 else -1.0,
                                              cap_a, j_up, j_dn, plan_lo, plan_hi,
                                              _SHAPER_WD_MUL * dur + _SHAPER_WD_PAD)
      if self.cruise_shaper_plan is not None:
        plan = self.cruise_shaper_plan
        gap = v_cruise - v_ego
        a_ff = plan.step(sample_now_ns, self.a_cruise, gap)
        # Checked EVERY frame (a fresh arm included): the monotone tap
        # clamp must never block a closing command -- an up-tap floor
        # at/above 0 while ego is past the set speed forbids the brake
        # side; a down-tap ceiling at/below 0 while ego is short of it
        # forbids the throttle side. Drop the plan the same frame.
        clamp_blocks_closing = (plan.a_lo >= 0.0 and gap < 0.0) or (plan.a_hi <= 0.0 and gap > 0.0)
        # A crossing past the arm threshold, either direction, any
        # phase, hands back to stock (subsumes the old 0.3 m/s
        # overshoot backstop: stricter and always armed).
        crossed = gap * plan.s < -_SHAPER_MIN_STEP
        # Clean handoff: gap inside the arm threshold and a_cruise
        # already near 0, so stock inherits no residue.
        settled = abs(gap) <= _SHAPER_MIN_STEP and abs(self.a_cruise) <= _SHAPER_SETTLE_A
        # Watchdog: a cap-starved plan (steep grade, live cap) must
        # not own the candidate forever.
        timed_out = (sample_now_ns - plan.t0_ns) / 1e9 > plan.t_wd
        if clamp_blocks_closing or crossed or settled or timed_out:
          self.cruise_shaper_plan = None
        else:
          # Enforced clamps bar only the wrong sign (up-plan never
          # brakes, down-plan never throttles); the RAW clamps stay on
          # the plan and drive every disarm check above. The law is
          # monotone in the closing direction from the tap-frame
          # command, so a down-tap command can never rise above the
          # tap-frame value (and with a_hi <= 0, eff_hi = 0 hard-clips
          # any positive command).
          eff_lo = min(plan.a_lo, 0.0)
          eff_hi = max(plan.a_hi, 0.0)
          shaper_cmd = (a_ff, eff_lo, eff_hi)
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
          print("  hint: shaper text detected; revert longitudinal_planner.py to the unshaped file before applying")
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
