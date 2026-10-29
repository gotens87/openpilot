#!/usr/bin/env python3
import math
import os
import time
from collections.abc import Callable
from dataclasses import replace
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.starpilot.longitudinal.encouragement import Encouragement
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (LongitudinalMpc, LongitudinalPlanSource, get_T_FOLLOW, get_jerk_factor,
                                                                                      qualified_model_leads, ModelLeadHistory, should_trigger_planner_fcw)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.navigation.intent import cruise_ceiling as navigation_ceiling
from openpilot.starpilot.car.tesla.preap_adaptive import startup_owner as preap_adaptive_owner
from openpilot.starpilot.longitudinal.vehicle_policy import forecast_should_stop, planner_cost_owner, far_follow_owner
from openpilot.starpilot.longitudinal.cruise_ceiling import CruiseCeiling, CurveCeiling, select_cruise_ceiling, select_curve_ceiling
from openpilot.starpilot.longitudinal.profile_runtime import (AppliedProfile, ProfileSmoother, ProfileTuning, SelectedProfileTuning,
                                                             TRAFFIC_CRUISE_BRAKE_MAGNITUDE)
from openpilot.starpilot.longitudinal.slc_coast import braking_lead_relevant
from openpilot.starpilot.longitudinal.throttle_gate import ModelThrottleGate
from openpilot.starpilot.longitudinal.ioniq6_start import eligible as ioniq6_long_eligible
from openpilot.starpilot.longitudinal.lane_change_gap import LaneChangeGap, project as project_lane_gap
from openpilot.starpilot.longitudinal.experimental_release import ConditionalHandoff, ExperimentalRelease, project as project_release
from openpilot.starpilot.longitudinal.force_stop import StopPlan
from openpilot.starpilot.longitudinal.stop_line import StopLine, horizon_from_path
from openpilot.starpilot.longitudinal.follow_jerk import FollowJerk
from openpilot.starpilot.longitudinal.lead_takeoff import LeadTakeoff, project as project_takeoff
from openpilot.starpilot.longitudinal.lead_behavior import adjust as adjust_lead_behavior
from openpilot.starpilot.longitudinal.hkg_vision_stop import owner_for as vision_stop_owner
from openpilot.starpilot.car.ford.stopped_lead import owner_for as ford_stop_owner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
from openpilot.starpilot.longitudinal.lead_approach import LeadApproach, LeadApproachKey, project as project_approach
from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
MIN_ALLOW_THROTTLE_SPEED = 2.5
NATIVE_ALLOW_THROTTLE_PROBABILITY = 0.4
NATIVE_PLAN_INPUTS = ('carControl', 'carState', 'controlsState', 'vehicleParameters',
                      'radarState', 'modelV2', 'selfdriveState')

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]
_TAKEOFF_TOTAL_BP = [0., 2., 5., 20., 40.]
_TAKEOFF_TOTAL_MAX = [3.5, 3.5, 1.7, 1.7, 3.2]

# /data/ops_patches/camry_cruise_shaper (read once at import):
# jerk-limited trapezoid in accel (S-curve in speed): RISE at j_up,
# HOLD at ceiling A, RELEASE at -j_dn off the LIVE gap (release when
# remaining <= a^2/(2*j_dn): command reaches 0 exactly as ego reaches
# set speed). Jerk scales with the arm-time gap: full j_up/j_dn at
# +4 km/h and up, linear below; +1 stays a ~2.3 s triangle (peak
# 0.24), +2/+3 hold the 0.35 small-step ceiling (grok must-fix) up
# to 90 km/h; past 100.8 km/h the small ceiling rises with the step
# (0.18 @ +2 -> 0.43 @ +4) so +2..+5 arrivals stay monotone; both
# laws blend to the table by +5, so +5/+9 keep the arrivals.
# Integrates the live a_cruise -> continuous by construction; no gain
# table, no pulse budget, no corr tail, no fixed duration to re-cap.
# Strict parse: any failure = None = stock.
CRUISE_SHAPER_MARKER = "/data/ops_patches/camry_cruise_shaper"
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


def _read_cruise_shaper():
  if not os.path.isfile(CRUISE_SHAPER_MARKER):
    return None
  try:
    with open(CRUISE_SHAPER_MARKER) as f:
      raw = f.read().strip()
    if not raw:
      cloudlog.info("camry_cruise_shaper: defaults j_up=%.2f j_dn=%.2f cap=%s decel=%.2f",
                    _SHAPER_J_UP, _SHAPER_J_DN, _SHAPER_CAP_V, _SHAPER_DECEL)
      return (_SHAPER_J_UP, _SHAPER_J_DN, list(_SHAPER_CAP_V), _SHAPER_DECEL)
    parts = raw.split(",")
    if not 1 <= len(parts) <= 4:
      raise ValueError(f"want 1..4 fields, got {len(parts)}")
    vals = [int(p) for p in parts]  # strict: floats, blanks, trailing commas all raise
    j_up = min(200, max(30, vals[0])) / 100.0
    j_dn = min(200, max(30, vals[1])) / 100.0 if len(vals) > 1 else _SHAPER_J_DN
    cap = [min(100, max(25, vals[2])) / 100.0] * len(_SHAPER_CAP_BP) if len(vals) > 2 else list(_SHAPER_CAP_V)
    decel = min(50, max(30, vals[3])) / 100.0 if len(vals) > 3 else _SHAPER_DECEL
    cloudlog.info("camry_cruise_shaper: marker %r -> j_up=%.2f j_dn=%.2f cap=%s decel=%.2f",
                  raw, j_up, j_dn, cap, decel)
    return (j_up, j_dn, cap, decel)
  except (OSError, ValueError) as e:
    cloudlog.warning("camry_cruise_shaper: marker parse failed (%s); stock cruise", e)
    return None


CRUISE_SHAPER = _read_cruise_shaper()

# Always on: geometric stop-line candidate for a committed empty red.
STOP_LINE_OFFSET_M = 6.0


class _ShaperPlan:
  """Jerk-limited trapezoid for one set-speed step; STATELESS law:
  step() integrates the live a_cruise and releases off the live gap,
  so a dropped frame, a live cap, the slew or actuator lag only
  stretch the hold, never corrupt the landing. a_lo/a_hi are the RAW
  monotone tap-frame clamps: they drive every disarm check in
  update(). The ENFORCED side only bars the wrong sign (up-plan never
  brakes below 0, down-plan never throttles above 0): the law is
  monotone in the closing direction from the tap-frame command, so
  nothing can pin the pull or the tail. t_wd is a watchdog deadline,
  not the profile."""
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


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                     profile: AppliedProfile | None = None,
                     base_brake_floor: float = A_CRUISE_MIN, selected_acceleration_max: float | None = None, faster_takeoffs: bool = False,
                     *, a_target_prev: float | None = None, shaper: tuple[float, float, float] | None = None):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)
  if selected_acceleration_max is not None and not e2e:
    max_accel = selected_acceleration_max
  if profile is not None:
    max_accel = min(max_accel, profile.acceleration_max, ACCEL_MAX)

  if selected_acceleration_max is not None:
    max_accel = min(max_accel, selected_acceleration_max, ACCEL_MAX)

  if not e2e:
    a_total_max = np.interp(v_ego, _TAKEOFF_TOTAL_BP, _TAKEOFF_TOTAL_MAX) if faster_takeoffs else np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  cruise_min = max(ACCEL_MIN, -profile.cruise_brake_magnitude) if profile is not None else max(ACCEL_MIN, base_brake_floor)
  if shaper is not None and CRUISE_SHAPER is not None and not e2e:
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
    target_accel = np.clip(v_cruise - v_ego, cruise_min, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  decel_start = a_cruise_prev if a_target_prev is None else min(a_cruise_prev, a_target_prev)
  target_accel = float(np.clip(target_accel, decel_start - j_cruise * dt, a_cruise_prev + j_cruise * dt))
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
  if shaper is not None and CRUISE_SHAPER is not None and not e2e:
    shaper_lo = max(cruise_min, -CRUISE_SHAPER[3], shaper[1])
    shaper_hi = min(max_accel, float(np.interp(v_ego, _SHAPER_CAP_BP, CRUISE_SHAPER[2])), shaper[2])
    if shaper_lo <= shaper_hi:
      post_lo, post_hi = shaper_lo, shaper_hi
  target_accel = float(np.clip(target_accel, post_lo, post_hi))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL, clock_ns: Callable[[], int] = time.monotonic_ns):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.model_lead_history = ModelLeadHistory()
    self.fcw = False
    self.dt = dt
    self.clock_ns = clock_ns
    self.allow_throttle = True
    self.accel_boost = AccelBoost()
    self.encouragement = Encouragement()
    self.plan_drive_id = 0
    self.throttle_gate = ModelThrottleGate()
    self.ioniq6_throttle_gate_enabled = ioniq6_long_eligible(CP)
    self.lane_change_gap = LaneChangeGap()
    self.experimental_release = ExperimentalRelease()
    self.lead_approach = LeadApproach(dt, CP.longitudinalActuatorDelay)
    self.cruise_shaper_plan = None
    self.cruise_shaper_v_cruise = None

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.last_cruise_ceiling_status = "absent"
    self.last_curve_ceiling_status = "absent"
    self.last_curve_ceiling_applied = False
    self.profile_smoother = ProfileSmoother()
    self.last_profile: AppliedProfile | None = None
    self.force_stop_plan = StopPlan()
    self.stop_line = StopLine(STOP_LINE_OFFSET_M)
    self.follow_jerk = FollowJerk(CP)
    self.vehicle_cost_owner = planner_cost_owner(CP)
    self.lead_takeoff = LeadTakeoff()
    self.vehicle_far_follow = far_follow_owner(CP, dt)
    self.vision_stop_buffer = vision_stop_owner(CP)
    self.ford_stop_approach = ford_stop_owner(CP, STOP_DISTANCE - .5)
    self.preap_adaptive = preap_adaptive_owner(CP)

  def update(self, sm, *, cruise_ceiling: CruiseCeiling | None = None,
             profile_tuning: ProfileTuning | None = None, curve_ceiling: CurveCeiling | None = None,
             curve_provider: Callable[[float | None], CurveCeiling | None] | None = None,
             traffic_mode: bool | None = False, lane_change_policy: LaneChangePolicy | None = None,
             global_braking_response: str | None = None, selected_profiles: SelectedProfileTuning | None = None,
             conditional_handoff: ConditionalHandoff | None = None,
             lead_approach_key: LeadApproachKey | None = None,
             force_stop_provider: Callable[[float], StopPlan] | None = None,
             faster_lead_takeoff: bool = False, takeoff_drive_id: int = 0,
             now_ns: int | None = None, drive_id: int = 0, wheel_coast=None, experimental_speed_nudge: bool = True):
    if curve_ceiling is not None and curve_provider is not None:
      raise ValueError('choose explicit Curve ceiling or same-cycle provider')
    if self.plan_drive_id != drive_id:
      self.model_lead_history.clear()
    self.plan_drive_id = drive_id
    faster_takeoffs = bool(faster_lead_takeoff)
    sample_now_ns = self.clock_ns() if now_ns is None else now_ns
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    ceiling = select_cruise_ceiling(
      cruise_ceiling, driver_v_cruise_kph=sm['carState'].vCruise, driver_cruise_mps=v_cruise,
      system_long_available=self.CP.openpilotLongitudinalControl,
      long_control_active=sm['controlsState'].longControlState != LongCtrlState.off,
      selfdrive_enabled=sm['selfdriveState'].enabled, force_decel=sm['controlsState'].forceDecel,
    )
    self.last_cruise_ceiling_status = ceiling.status
    if ceiling.effective_mps is not None:
      v_cruise = ceiling.effective_mps
    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    profile_eligible = (self.CP.openpilotLongitudinalControl and
                        sm['selfdriveState'].enabled and not long_control_off and sm['carControl'].longActive)
    if profile_eligible:
      self.last_profile = self.profile_smoother.sample(profile_tuning, sm['selfdriveState'].personality, self.dt,
                                                       traffic_mode=traffic_mode)
    else:
      self.profile_smoother.applied = None
      self.last_profile = None

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    if self.ioniq6_throttle_gate_enabled and sm['carControl'].longActive and not reset_state:
      model_ns = getattr(sm, 'logMonoTime', {}).get('modelV2', 0)
      self.allow_throttle = self.throttle_gate.step(throttle_prob if len(throttle_probs) > 1 else float('nan'),
                                                    v_ego, low_speed_mps=MIN_ALLOW_THROTTLE_SPEED, model_ns=model_ns,
                                                    now_ns=sample_now_ns,
                                                    model_valid=(getattr(sm, 'valid', {}).get('modelV2', False) is True and
                                                                 getattr(sm, 'alive', {}).get('modelV2', False) is True))
    else:
      self.throttle_gate.reset()
      self.allow_throttle = throttle_prob > NATIVE_ALLOW_THROTTLE_PROBABILITY or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      self.cruise_shaper_plan = None  # grok3 must-fix 3: the plan dies with the state reset, not one gate check later

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    base_follow = self.last_profile.follow_seconds if self.last_profile is not None else get_T_FOLLOW(sm['selfdriveState'].personality)
    approach_follow = self.lead_approach.step(
      lead_approach_key, project_approach(sm, self.CP, sample_now_ns, lead_approach_key) if lead_approach_key is not None else None,
      base_follow, blocked=reset_state)
    gap_follow = self.lane_change_gap.step(
      lane_change_policy, project_lane_gap(sm, sample_now_ns) if lane_change_policy is not None else None,
      base_follow=base_follow, long_active=bool(self.CP.openpilotLongitudinalControl and sm['carControl'].longActive and
                                                not reset_state and sm['selfdriveState'].enabled and approach_follow is None))
    follow = approach_follow if approach_follow is not None else gap_follow
    selected_follow = follow if follow is not None else base_follow
    self.force_stop_plan = force_stop_provider(selected_follow) if force_stop_provider is not None else StopPlan()
    if (reset_state or not profile_eligible or self.force_stop_plan.model_ns != getattr(sm, 'logMonoTime', {}).get('modelV2') or
        sm['carState'].gasPressed or sm['carState'].brakePressed or sm['controlsState'].forceDecel):
      self.force_stop_plan = StopPlan()
    if self.force_stop_plan.speed_ceiling_mps is not None:
      v_cruise = min(v_cruise, self.force_stop_plan.speed_ceiling_mps)
    follow_scale = self.follow_jerk.sample(sm, self.CP, sample_now_ns, selected_follow, self.force_stop_plan,
                                           active=profile_eligible and not reset_state, drive_id=drive_id)
    accelerating = sm['carState'].aEgo >= 0
    weights = None
    if self.vehicle_cost_owner is not None:
      # Current mode arbitration is retained; do not recreate old one-cycle delay.
      weights = self.vehicle_cost_owner.sample(
        sm, sample_now_ns, active=profile_eligible and not reset_state, drive_id=drive_id,
        mode='blended' if sm['selfdriveState'].experimentalMode else 'acc',
        acceleration_factor=(self.last_profile.acceleration_jerk if accelerating else self.last_profile.deceleration_jerk)
        if self.last_profile is not None else get_jerk_factor(sm['selfdriveState'].personality),
        speed_factor=(self.last_profile.speed_jerk if accelerating else self.last_profile.speed_decrease_jerk)
        if self.last_profile is not None else get_jerk_factor(sm['selfdriveState'].personality),
        danger_factor=self.last_profile.danger_jerk if self.last_profile is not None else get_jerk_factor(sm['selfdriveState'].personality),
        stop_plan=self.force_stop_plan, follow_scale=follow_scale, prev_accel_constraint=prev_accel_constraint)
    if weights is not None:
      self.mpc.set_cost_weights(weights.stage, weights.constraints)
    else:
      self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality,
                           acceleration_jerk=(self.last_profile.acceleration_jerk if accelerating else self.last_profile.deceleration_jerk)
                           if self.last_profile is not None else None,
                           speed_jerk=(self.last_profile.speed_jerk if accelerating else self.last_profile.speed_decrease_jerk)
                           if self.last_profile is not None else None,
                           danger_jerk=self.last_profile.danger_jerk if self.last_profile is not None else None,
                           stop_jerk_scale=self.force_stop_plan.jerk_scale, acceleration_change_scale=follow_scale)
    selected_acceleration = selected_profiles.acceleration_max if profile_eligible and selected_profiles is not None else None
    acceleration_cap = self.last_profile.acceleration_max if self.last_profile is not None else None
    if selected_acceleration is not None:
      acceleration_cap = selected_acceleration
    preap_acceleration_cap = None
    if self.preap_adaptive is not None:
      preap_acceleration_cap = self.preap_adaptive.sample(
        sm, now_ns=sample_now_ns, drive_id=drive_id, upper=ACCEL_MAX if acceleration_cap is None else acceleration_cap,
        follow_seconds=selected_follow, active=profile_eligible and not reset_state)
      if preap_acceleration_cap is not None:
        acceleration_cap = preap_acceleration_cap
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality,
                    follow_seconds=follow if follow is not None else
                    (self.last_profile.follow_seconds if self.last_profile is not None else None),
                    acceleration_max=acceleration_cap,
                    stop_line_m=self.force_stop_plan.obstacle_m, modelV2=qualified_model_leads(sm, sample_now_ns, self.model_lead_history))

    # MPC has selected the actual follow time for this model cycle. Its solve
    # does not use the Curve cruise cap, so a provider may now evaluate the
    # filtered lead condition without reading a stale saved preference.
    if curve_provider is not None:
      try:
        selected_follow = float(self.mpc.params[0, 4])
        if not math.isfinite(selected_follow) or not 0.5 <= selected_follow <= 3.0:
          selected_follow = None
      except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        selected_follow = None
      curve_ceiling = curve_provider(selected_follow)
    curve = select_curve_ceiling(
      curve_ceiling, model_ns=getattr(sm, 'logMonoTime', {}).get('modelV2', 0),
      driver_v_cruise_kph=sm['carState'].vCruise, effective_cruise_mps=v_cruise,
      system_long_available=self.CP.openpilotLongitudinalControl,
      long_control_active=sm['controlsState'].longControlState != LongCtrlState.off,
      car_long_active=sm['carControl'].longActive, selfdrive_enabled=sm['selfdriveState'].enabled,
      force_decel=sm['controlsState'].forceDecel, driver_override=sm['carState'].gasPressed or sm['carState'].brakePressed,
    )
    self.last_curve_ceiling_status = curve.status
    self.last_curve_ceiling_applied = False
    if curve.effective_mps is not None:
      v_cruise = curve.effective_mps
    navigation_target = navigation_ceiling(sm, self.CP, sample_now_ns, v_cruise) if profile_eligible else None
    if navigation_target is not None:
      v_cruise = min(v_cruise, navigation_target)
    coast_plan = None
    if wheel_coast is not None:
      owner, wheel_params = wheel_coast
      pulse_lead = any(lead.present and (v_ego - lead.vLead > 0.5 or lead.aLeadK < -0.4 or
                                        lead.dRel < max(18.0, 2.0 * v_ego))
                       for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo))
      coast_plan = owner.sample(wheel_params, self.CP, sm, now_ns=sample_now_ns, target_mps=v_cruise,
        lead_relevant=bool(pulse_lead), stop_context=bool(sm['carState'].standstill or sm['controlsState'].forceDecel or
        self.force_stop_plan.forcing or sm['modelV2'].action.shouldStop or not self.allow_throttle))
      if coast_plan.speed_ceiling_mps is not None:
        v_cruise = min(v_cruise, coast_plan.speed_ceiling_mps)
    if sm['controlsState'].forceDecel or coast_plan is not None and coast_plan.force_decel:
      v_cruise = 0.0

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.CP.carFingerprint == 'SUBARU_CROSSTREK_2025':
      self.fcw = self.fcw and should_trigger_planner_fcw(sm['radarState'].leadOne, float(v_ego))
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = forecast_should_stop(
      self.CP, self.v_desired_trajectory, CONTROL_N_T_IDX, action_t,
      fallback=should_stop(v_ego, output_a_target_mpc))
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    relevant_lead = any(braking_lead_relevant(lead, v_ego) for lead in
                        (sm['radarState'].leadOne, sm['radarState'].leadTwo))
    global_magnitude = ({'standard': 1.2, 'eco': 0.6, 'sport': 2.4}.get(global_braking_response)
                        if type(global_braking_response) is str else None)
    selected_braking = (selected_profiles.cruise_brake_magnitude
                        if profile_eligible and selected_profiles is not None else None)
    if selected_braking is not None:
      global_magnitude = selected_braking  # One resolved response, never multiplied or stacked.
      global_braking_response = selected_profiles.braking_style
    global_selected = (global_magnitude is not None and profile_eligible and (traffic_mode is False or selected_braking is not None) and
                       not sm['controlsState'].forceDecel and not sm['carState'].standstill and
                       not sm['carState'].brakePressed and not sm['carState'].gasPressed and
                       not relevant_lead and not self.force_stop_plan.forcing and not sm['modelV2'].action.shouldStop and
                       not output_should_stop_mpc and
                       (selected_braking is not None or self.last_profile is None or profile_tuning is None or
                        profile_tuning.cruise_brake_magnitude is None))
    global_floor = -global_magnitude if global_selected else A_CRUISE_MIN
    cruise_profile = self.last_profile
    custom_traffic_braking = (traffic_mode is True and profile_eligible and cruise_profile is not None and
                              profile_tuning is not None and profile_tuning.traffic_braking_custom is True and
                              not sm['controlsState'].forceDecel)
    if custom_traffic_braking and (sm['carState'].standstill or sm['radarState'].leadOne.present or
                                   sm['radarState'].leadTwo.present):
      cruise_profile = replace(cruise_profile, cruise_brake_magnitude=TRAFFIC_CRUISE_BRAKE_MAGNITUDE)
      self.profile_smoother.applied = cruise_profile
      self.last_profile = cruise_profile
    if global_selected and cruise_profile is not None:
      cruise_profile = replace(cruise_profile, cruise_brake_magnitude=global_magnitude)
    if sm['controlsState'].forceDecel:
      cruise_profile = None
    elif (ceiling.effective_mps is not None or curve.effective_mps is not None) and cruise_profile is not None:
      # An eligible lower ceiling must retain at least native approach
      # braking even when the saved profile requests a softer cruise response.
      cruise_profile = replace(cruise_profile, cruise_brake_magnitude=max(cruise_profile.cruise_brake_magnitude,
                                                                          abs(A_CRUISE_MIN)))
    if ceiling.effective_mps is not None or curve.effective_mps is not None:
      global_floor = min(global_floor, A_CRUISE_MIN)
    if selected_acceleration is not None and cruise_profile is not None:
      # The selected curve owns acceleration; following/jerks retain their owner.
      cruise_profile = replace(cruise_profile, acceleration_max=selected_acceleration)
    if coast_plan is not None and coast_plan.brake_floor is not None:
      global_floor = coast_plan.brake_floor
      cruise_profile = None
    if preap_acceleration_cap is not None:
      selected_acceleration = (preap_acceleration_cap if selected_acceleration is None else
                               min(selected_acceleration, preap_acceleration_cap))
    # Marker-gated shaped set-speed response: jerk-limited trapezoid
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
          # by +5 (+5 and up unchanged); a flat marker cap below the
          # ceiling still wins. Continuous in gap and speed.
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
    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, cruise_profile,
                                     base_brake_floor=global_floor, selected_acceleration_max=selected_acceleration, faster_takeoffs=faster_takeoffs,
                                     a_target_prev=self.output_a_target, shaper=shaper_cmd)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    encouraged_model = self.encouragement.update(
      sm, self.CP, now_ns=sample_now_ns, drive_id=drive_id, dt=self.dt, target_speed=v_cruise,
      model_accel=output_a_target_e2e, mpc_accel=output_a_target_mpc, cruise_accel=self.a_cruise,
      lateral_accel=(v_ego ** 2 * steer_angle_without_offset * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)
                     if self.CP.steerRatio > 0 and self.CP.wheelbase > 0 else math.inf),
      blocked=bool(experimental_speed_nudge is not True or reset_state or self.fcw or not self.allow_throttle or self.force_stop_plan.forcing or
                   output_should_stop_mpc or cruise_should_stop or self.mpc.solution_status != 0 or
                   curve.effective_mps is not None or traffic_mode is not False))
    manual_model = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)
    output_a_target_e2e = max(manual_model, encouraged_model)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    lead_one = sm['radarState'].leadOne
    stop_line_accel = self.stop_line.step(
      self.dt, v_ego, horizon_from_path(sm['modelV2'].position.x),
      stop_committed=(None if wheel_coast is None else bool(getattr(wheel_coast[0], 'conditional_stop', False) is True)),
      experimental=bool(sm['selfdriveState'].experimentalMode), enabled=not reset_state,
      lead_relevant=bool(relevant_lead), lead_drel=float(lead_one.dRel) if lead_one.present else None,
      force_stop=bool(self.force_stop_plan.forcing or self.force_stop_plan.obstacle_m is not None),
      pedal=bool(sm['carState'].gasPressed or sm['carState'].brakePressed),
      standstill=bool(sm['carState'].standstill), model_should_stop=bool(output_should_stop_e2e),
      a_prev=float(a_prev))
    if stop_line_accel is not None:
      candidates.append((stop_line_accel, LongitudinalPlanSource.cruise, False))
    if sm['selfdriveState'].experimentalMode:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    mpc_source = self.mpc.source
    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    # A curve candidate alone cannot own driver feedback. Lead/e2e braking and
    # an equal or lower SLC ceiling retain their own attribution.
    self.last_curve_ceiling_applied = (curve.effective_mps is not None and self.a_cruise < output_a_target_mpc and
                                      (not sm['selfdriveState'].experimentalMode or self.a_cruise < output_a_target_e2e))
    self.output_should_stop = self.force_stop_plan.should_stop or any(should_stop for _, _, should_stop in candidates)
    if conditional_handoff is None:
      self.experimental_release.reset()
    else:
      mpc_lead_index = {LongitudinalPlanSource.lead0: 0, LongitudinalPlanSource.lead1: 1}.get(mpc_source)
      release_cap = self.experimental_release.step(
        conditional_handoff, project_release(sm, self.CP, sample_now_ns, lead_index=mpc_lead_index,
                                             conditional_handoff=conditional_handoff),
        previous_target=a_prev, target=output_a_target, follow_seconds=float(self.mpc.params[0, 4]),
        blocked=bool(reset_state or self.output_should_stop or self.fcw))
      if release_cap is not None:
        output_a_target = min(output_a_target, release_cap)
    output_a_target = adjust_lead_behavior(
      target=output_a_target, sm=sm, cp=self.CP, now_ns=sample_now_ns, key=conditional_handoff,
      follow_seconds=float(self.mpc.params[0, 4]), mpc_target=output_a_target_mpc,
      cruise_target=self.a_cruise, model_target=output_a_target_e2e, stopping=self.output_should_stop,
      force_stop=self.force_stop_plan.forcing, traffic_mode=traffic_mode, accel_min=ACCEL_MIN)
    if faster_lead_takeoff:
      # Dom takeoff bypasses the standstill cruise slew, never its final bounds.
      takeoff_max = ACCEL_MAX if sm['selfdriveState'].experimentalMode else get_max_accel(v_ego)
      if selected_acceleration is not None and not sm['selfdriveState'].experimentalMode:
        takeoff_max = selected_acceleration
      if cruise_profile is not None:
        takeoff_max = min(takeoff_max, cruise_profile.acceleration_max)
      if selected_acceleration is not None:
        takeoff_max = min(takeoff_max, selected_acceleration)
      a_total_max = np.interp(v_ego, _TAKEOFF_TOTAL_BP, _TAKEOFF_TOTAL_MAX) if faster_takeoffs else np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
      a_y = v_ego ** 2 * steer_angle_without_offset * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)
      takeoff_max = min(takeoff_max, math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.)), max(0., v_cruise - v_ego))
      takeoff_frame = project_takeoff(
        sm, self.CP, now_ns=sample_now_ns, drive_id=takeoff_drive_id, stop_plan=self.force_stop_plan,
        acceleration_max=takeoff_max, mpc_accel=output_a_target_mpc, follow_seconds=float(self.mpc.params[0, 4]),
        blocked=bool(experimental_speed_nudge is not True or reset_state or self.fcw or not self.allow_throttle or traffic_mode is not False or
                     self.mpc.solution_status != 0))
      output_a_target, self.output_should_stop = self.lead_takeoff.step(
        True, takeoff_frame, output_a_target, self.output_should_stop)
    else:
      self.lead_takeoff.reset()
    if self.vehicle_far_follow is not None:
      output_a_target = self.vehicle_far_follow.sample(
        sm, now_ns=sample_now_ns, drive_id=drive_id, active=profile_eligible and not reset_state,
        target=float(output_a_target), previous_target=float(a_prev), follow_seconds=float(self.mpc.params[0, 4]),
        blocked=bool(self.output_should_stop or self.fcw or self.force_stop_plan.forcing or
                     self.force_stop_plan.approach_distance_m > 0 or self.mpc.solution_status != 0))
    if self.vision_stop_buffer is not None:
      output_a_target, self.output_should_stop = self.vision_stop_buffer.sample(
        sm, now_ns=sample_now_ns, drive_id=drive_id, target=float(output_a_target),
        stopping=self.output_should_stop, accel_min=ACCEL_MIN,
        active=profile_eligible and not reset_state and self.mpc.solution_status == 0)
    if preap_acceleration_cap is not None:
      output_a_target = min(output_a_target, preap_acceleration_cap)
    if self.ford_stop_approach is not None:
      output_a_target, self.output_should_stop = self.ford_stop_approach.sample(
        sm, now_ns=sample_now_ns, drive_id=drive_id, target=float(output_a_target),
        stopping=self.output_should_stop, accel_min=ACCEL_MIN, active=profile_eligible and not reset_state)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(NATIVE_PLAN_INPUTS)

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime - sm.logMonoTime['modelV2']) / 1e9
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    companion = messaging.new_message('starpilotLongitudinalPlan')
    companion.logMonoTime = plan_send.logMonoTime
    companion.valid = bool(plan_send.valid and 0 < self.plan_drive_id < longitudinalPlan.modelMonoTime <= plan_send.logMonoTime)
    hold = companion.starpilotLongitudinalPlan
    hold.version = 1
    hold.sourcePlanMonoTime = plan_send.logMonoTime
    hold.modelMonoTime = longitudinalPlan.modelMonoTime
    hold.driveStartMonoTime = self.plan_drive_id
    hold.forceStopHolding = bool(self.force_stop_plan.manual_hold)
    pm.send('longitudinalPlan', plan_send)
    pm.send('starpilotLongitudinalPlan', companion)
