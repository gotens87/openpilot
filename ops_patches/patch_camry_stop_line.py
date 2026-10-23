#!/usr/bin/env python3
# Steady stop-line braking at red lights. Once Conditional Experimental Mode
# has committed to a stop (StopLightDetector.committed, experimentalMode true)
# with no lead, the planner's only brake is the model's raw
# desiredAcceleration: soft for the first ~60 m, hard in the last 20, aimed at
# the far side of the crosswalk. With /data/ops_patches/camry_stop_line present
# (read once at import) the planner adds one more min() candidate
# (openpilot/starpilot/longitudinal/stop_line.py): target = model stop horizon
# (modelV2.position.x endpoint) - offset, dead-reckoned every frame and only
# ever re-anchored shorter; a constant decel that reaches 2.5 m/s 6.9 m before
# the target, then a ~0.45 m/s^2 finish; slew-limited off the previous
# selected output; both laws closed loop on live speed and distance. Every
# stock candidate, e2e included, stays in the min(), so the output is
# min(stock, candidate): never softer than stock. Arms once per CEM stop
# commitment (plannerd's wheel_coast.conditional_stop edge) when:
# experimentalMode, long control on, v < 20 m/s, horizon finite and < 150 m,
# no braking-relevant lead and leadOne not inside target + 10 m, no ForceStops
# obstacle, no pedal, not standstill. Drops on green (horizon beyond target +
# 30 m with model shouldStop false for 0.5 s), experimentalMode off 0.5 s or
# standstill, and then waits for the next commitment; drops on pedal,
# ForceStops, disengage or v > 22 m/s without forgetting the current commitment. A lead
# mid-approach hands the frame back to stock while the commit dead-reckons.
# Marker contents: empty = 6.0 m; else integer cm 0..1200 (bad value or
# unreadable file = stock with a log line). Absent marker = stock planner
# behaviour exactly. Lead stops, long_mpc.py, longcontrol.py and acados are
# untouched. Also writes the pure module and its pytest file; neither needs
# acados, cereal or Params.
import os
import sys

PLANNER_PATH = "/data/openpilot/openpilot/selfdrive/controls/lib/longitudinal_planner.py"
MODULE_PATH = "/data/openpilot/openpilot/starpilot/longitudinal/stop_line.py"
TEST_PATH = "/data/openpilot/openpilot/starpilot/longitudinal/tests/test_stop_line.py"
TEST_INIT_PATH = "/data/openpilot/openpilot/starpilot/longitudinal/tests/__init__.py"

MODULE = r'''"""Geometric stop-line braking candidate for a committed red-light stop.

Pure state machine: plain floats and bools in, one extra planner accel
candidate out (None = nothing added this frame). No openpilot imports, so the
planner hook and its tests share this module without acados, cereal or Params.

The candidate only ever joins the planner's min(); every stock candidate,
the raw e2e desiredAcceleration included, stays in. The output is therefore
min(stock, candidate): never softer than stock, and the model keeps authority
over anything only it can see.

Arming needs a fresh Conditional Experimental Mode stop commitment
(StopLightDetector.committed rising edge, `stop_committed`), experimentalMode,
long control on, v < V_ARM_MAX_MPS, a finite model stop horizon inside
HORIZON_MAX_M, no braking-relevant lead and leadOne not inside target +
LEAD_ONE_MARGIN_M. One commitment arms at most once. A green clear, experimental-off dwell or
standstill drops the target; pedal, ForceStops, disengage or v > V_CLEAR_MPS
do the same. Those drops keep the current commitment high, so gas after a
green cannot look like a new edge. The next arm waits for stop_committed to
go false then true. Missing commitment evidence (`stop_committed is None`)
skips the frame and does not count as a detector false.

Target = horizon - offset, dead-reckoned every frame (d -= v * dt), re-anchored
only shorter and only on frames the stop line owns (a lead-owned frame keeps
dead-reckoning but ignores the lead-induced horizon). The law holds the
constant decel that reaches V_SOFT_MPS exactly D_SOFT_M before an aim point
STOP_MARGIN_M short of the target (firm phase), then eases at about A_SOFT
(finish phase). Both laws are closed loop on live speed and distance,
projected LAG_S ahead using the previous selected planner output as the
acceleration in the actuator (a command-based approximation, validated in the
tests across 0.15-0.3 s lags). Holding -v^2/(2d) to the target and clipping
the last metres softer stopped the first design ~3.5 m past the target;
reserving D_SOFT_M for the finish is the fix. Slew limits reference the
previous selected output, so a frame another candidate won is released from
where that candidate left the car, at J_RELEASE.
"""

import math

MARKER_DEFAULT_OFFSET_M = 6.0   # empty marker: stop this far short of the model stop horizon, m
MARKER_MAX_CM = 1200            # marker integer range, cm
MARKER_MAX_CHARS = 8            # longer markers are rejected before int()
MARKER_READ_LIMIT = 64          # marker file bytes; one extra byte means over-limit -> stock
PATH_POINTS = 33                # modelV2.position.x length (ModelConstants.IDX_N)
PATH_MAX_M = 500.0              # model path endpoint sanity bound, m

V_ARM_MAX_MPS = 20.0            # never arm at or above this (72 km/h)
V_CLEAR_MPS = 22.0              # clear above this
HORIZON_MAX_M = 150.0           # arm only when the model stop horizon is finite and inside this, m
LEAD_ONE_MARGIN_M = 10.0        # leadOne closer than target + this hands the frame to stock, m
TARGET_FLOOR_M = 2.0            # arm floor on horizon - offset, m
EXPERIMENTAL_OFF_CLEAR_S = 0.5  # experimentalMode false this long drops the target, s
GREEN_CLEAR_S = 0.5             # horizon past target + margin with model shouldStop false this long drops it, s
GREEN_HORIZON_MARGIN_M = 30.0   # m

V_SOFT_MPS = 2.5                # hand-off speed to the soft finish (9 km/h)
A_SOFT = 0.45                   # finish decel the firm phase hands off to, m/s^2
D_SOFT_M = V_SOFT_MPS ** 2 / (2.0 * A_SOFT)  # 6.94 m reserved for the finish
STOP_MARGIN_M = 0.5             # the laws aim this far short of the target, m
TAIL_M = 1.0                    # inside this (projected) the finish holds at least A_SOFT, m
V_FAST_MPS = 15.0 / 3.6         # above this the candidate never exceeds A_FIRM_MAX
A_FIRM_MAX = 1.35               # cap above V_FAST_MPS, m/s^2
A_FINISH_MAX = 2.0              # cap below V_FAST_MPS, m/s^2; only a hot (late-seen) arrival reaches it
A_FINISH_MIN = 0.15             # finish floor inside D_SOFT_M, m/s^2
FIRM_DENOM_FLOOR_M = 1.0        # firm law gain limit near the hand-off point, m
FINISH_DENOM_FLOOR_M = 0.5      # finish law gain limit near the aim point, m
LAG_S = 0.2                     # actuator lag the laws project over, s
J_BRAKE = 2.0                   # max rate of brake increase from the previous selected output, m/s^3
J_RELEASE = 1.0                 # max rate of brake release from the previous selected output, m/s^3


def parse_marker(raw):
  """None (marker absent) -> None; '' -> default offset; integer cm 0..MARKER_MAX_CM -> metres; else None."""
  if raw is None:
    return None
  text = raw.strip()
  if not text:
    return MARKER_DEFAULT_OFFSET_M
  if len(text) > MARKER_MAX_CHARS or not (text.isascii() and text.isdigit()):
    return None
  cm = int(text)
  if cm > MARKER_MAX_CM:
    return None
  return cm / 100.0


def parse_marker_bytes(raw_b):
  """None if missing, over MARKER_READ_LIMIT, or invalid; else parse_marker of the decoded text."""
  if raw_b is None or len(raw_b) > MARKER_READ_LIMIT:
    return None
  return parse_marker(raw_b.decode("utf-8", "replace"))


def horizon_from_path(path):
  """Model stop horizon: the endpoint of a finite, non-decreasing PATH_POINTS-long position.x; else None."""
  try:
    xs = [float(x) for x in path]
  except (TypeError, ValueError):
    return None
  if len(xs) != PATH_POINTS or not all(math.isfinite(x) for x in xs):
    return None
  if xs[0] < 0.0 or xs[-1] > PATH_MAX_M or any(b < a for a, b in zip(xs, xs[1:])):
    return None
  return xs[-1]


class StopLine:
  def __init__(self, offset_m):
    self.offset_m = float(offset_m)
    self.reset()

  def reset(self):
    self.target_m = None        # dead-reckoned distance to the stop target, m; None = not armed
    self.pending = False        # a commitment edge seen and not yet armed or dropped
    self.committed_prev = False
    self.exp_off_s = 0.0
    self.green_s = 0.0

  @property
  def active(self):
    return self.target_m is not None

  def _drop(self):
    """Drop the target and consume the commitment: no re-arm until the next commitment edge."""
    self.target_m = None
    self.pending = False
    self.green_s = 0.0

  def step(self, dt, v_ego, horizon_m, *, stop_committed, experimental, enabled, lead_relevant, lead_drel,
           force_stop, pedal, standstill, model_should_stop, a_prev):
    if stop_committed is None:
      return None  # missing evidence this frame: not a detector false
    rising = stop_committed and not self.committed_prev
    self.committed_prev = stop_committed
    if rising:
      self.pending = True
    if not stop_committed:
      self.reset()
      return None
    if not enabled or pedal or force_stop or v_ego > V_CLEAR_MPS:
      self._drop()
      return None
    if standstill:
      self._drop()
      return None
    if experimental:
      self.exp_off_s = 0.0
    else:
      self.exp_off_s += dt
      if self.exp_off_s > EXPERIMENTAL_OFF_CLEAR_S - 0.5 * dt:
        self._drop()
        return None
    if self.target_m is None:
      if (not self.pending or not experimental or v_ego >= V_ARM_MAX_MPS or horizon_m is None or
          horizon_m >= HORIZON_MAX_M or lead_relevant):
        return None
      target = max(horizon_m - self.offset_m, TARGET_FLOOR_M)
      if lead_drel is not None and lead_drel < target + LEAD_ONE_MARGIN_M:
        return None
      self.target_m = target
      self.pending = False
      self.green_s = 0.0
      lead_owned = False
    else:
      self.target_m = max(self.target_m - v_ego * dt, 0.0)
      lead_owned = lead_relevant or (lead_drel is not None and lead_drel < self.target_m + LEAD_ONE_MARGIN_M)
      if horizon_m is not None and not lead_owned:
        self.target_m = min(self.target_m, max(horizon_m - self.offset_m, 0.0))
    if lead_owned:
      self.green_s = 0.0
      return None  # stock owns this frame (lead stop); the commit keeps dead-reckoning
    if horizon_m is not None and not model_should_stop and horizon_m > self.target_m + GREEN_HORIZON_MARGIN_M:
      self.green_s += dt
      if self.green_s > GREEN_CLEAR_S - 0.5 * dt:
        self._drop()
        return None
    else:
      self.green_s = 0.0
    a = self.law(v_ego, self.target_m, a_prev)
    return min(max(a, a_prev - J_BRAKE * dt), a_prev + J_RELEASE * dt)

  @staticmethod
  def law(v_ego, d_target, a_now):
    """Unlimited candidate for speed v_ego at d_target from the stop, with a_now in the actuator."""
    v = max(v_ego + a_now * LAG_S, 0.0)
    d = d_target - STOP_MARGIN_M - max(v_ego, 0.0) * LAG_S - 0.5 * a_now * LAG_S * LAG_S
    cap = A_FIRM_MAX if v_ego > V_FAST_MPS else A_FINISH_MAX
    if d > D_SOFT_M and v > V_SOFT_MPS:
      a = -(v * v - V_SOFT_MPS ** 2) / (2.0 * max(d - D_SOFT_M, FIRM_DENOM_FLOOR_M))
      return max(a, -A_FIRM_MAX)
    a = -(v * v) / (2.0 * max(d, FINISH_DENOM_FLOOR_M))
    if d > D_SOFT_M:
      return min(max(a, -cap), 0.0)  # slow and far: ease geometrically to the aim point
    if d <= TAIL_M:
      a = min(a, -A_SOFT)  # the last metre never goes softer than the finish decel
    return min(max(a, -cap), -A_FINISH_MIN)
'''

TEST = r'''"""Stop-line candidate: marker parse, commitment gating, shrink-only target, profiles and arbitrated approaches."""
import math

import pytest

from openpilot.starpilot.longitudinal import stop_line as sl
from openpilot.starpilot.longitudinal.stop_line import StopLine, horizon_from_path, parse_marker, parse_marker_bytes

DT = 0.05
KMH = 1.0 / 3.6
STOP_RAMP = 0.3     # LongControl stopping ramp with marker camry_stop_ramp, m/s^3
STOP_ACCEL = -2.0   # CP.stopAccel
V_FAST = 15.0 * KMH
EPS = 1e-9


def should_stop(v_ego, a_target):
  return v_ego < 0.3 and a_target < 0.1


def frame(**overrides):
  base = dict(dt=DT, v_ego=13.0, horizon_m=96.0, stop_committed=True, experimental=True, enabled=True,
              lead_relevant=False, lead_drel=None, force_stop=False, pedal=False, standstill=False,
              model_should_stop=False, a_prev=0.0)
  base.update(overrides)
  return base


def run(line, n, **overrides):
  """Step n frames; unless a_prev is given, the candidate's own output is the previous selected output."""
  out = None
  a_prev = overrides.get('a_prev', getattr(line, 'test_a_prev', 0.0))
  for _ in range(n):
    kw = frame(**overrides)
    kw['a_prev'] = a_prev
    out = line.step(kw.pop('dt'), kw.pop('v_ego'), kw.pop('horizon_m'), **kw)
    if 'a_prev' not in overrides and out is not None:
      a_prev = out
  line.test_a_prev = a_prev
  return out


def late_model(v, d_model):
  """A y6-shaped model: nothing until 60 m from its endpoint, then a constant-decel stop at it."""
  if d_model > 60.0:
    return 0.0
  return max(-3.5, -v * v / (2.0 * max(d_model - 1.0, 0.5)))


def simulate(v0_kmh, horizon_m, offset=6.0, lag_s=0.2, candidate=True, model=None, model_stop_below=None,
             max_s=90.0, hold_frames=20):
  """Planner-arbitrated approach at 20 Hz through a first-order actuator and LongControl's state machine.

  The model endpoint sits `offset` past the line, so the candidate's target is the line. Stock
  candidates: MPC/cruise at 0 and, when `model` is given, its e2e accel (shouldStop below
  `model_stop_below`). The winner is min() over stock plus the candidate; stopping follows
  should_stop/any() like longcontrol (entered and left), and at standstill the CEM hold forces
  shouldStop. Runs `hold_frames` past standstill.
  """
  line = StopLine(offset) if candidate else None
  v = v0_kmh * KMH
  d_line = horizon_m - offset
  a_act = a_out = 0.0
  stopping = False
  min_fast = 0.0
  last8 = []
  soft_at = None
  short_m = None
  held = 0
  for _ in range(int(max_s / DT)):
    standstill = v <= 0.0
    d_model = d_line + offset
    model_stop = v < 0.3 or (model_stop_below is not None and v < model_stop_below)
    cands = [(0.0, should_stop(v, 0.0))]  # MPC fake lead / cruise at set speed
    if model is not None:
      cands.append((model(v, d_model), model_stop))
    stock_min = min(a for a, _ in cands)
    cand = None
    if line is not None:
      cand = line.step(DT, v, d_model, stop_committed=True, experimental=True, enabled=True, lead_relevant=False,
                       lead_drel=None, force_stop=False, pedal=False, standstill=standstill,
                       model_should_stop=model_stop, a_prev=a_out)
      if cand is not None:
        cands.append((cand, False))  # accel-only: stock owns shouldStop / LongControl stopping
    selected = min(a for a, _ in cands)
    assert selected <= stock_min, "the candidate can only lower the output"
    flag = any(f for _, f in cands)
    if standstill:
      flag, selected = True, min(selected, 0.0)  # CEM standstill hold
      assert cand is None and (line is None or not line.active)
      held += 1
      if held >= hold_frames:
        break
    stopping = flag
    a_out = max(min(a_out, 0.0) - STOP_RAMP * DT, STOP_ACCEL) if stopping else selected
    if v > V_FAST:
      min_fast = min(min_fast, a_out)
    if 0.0 < d_line <= 8.0:
      last8.append(a_out)
    if soft_at is None and v <= sl.V_SOFT_MPS:
      soft_at = d_line
    a_act += (a_out - a_act) * DT / lag_s
    v_new = max(v + a_act * DT, 0.0)
    d_line -= 0.5 * (v + v_new) * DT
    v = v_new
    if v == 0.0 and short_m is None:
      short_m = d_line
  assert short_m is not None, "did not stop"
  assert d_line == short_m, "rolled after standstill"
  return dict(short_m=short_m, min_fast=min_fast, last8_mean=sum(last8) / max(len(last8), 1), soft_at_m=soft_at)


def required_decel(v0_kmh, horizon_m, offset=6.0):
  v0 = v0_kmh * KMH
  return (v0 * v0 - sl.V_SOFT_MPS ** 2) / (2.0 * (horizon_m - offset - sl.D_SOFT_M))


# --- marker ---------------------------------------------------------------

@pytest.mark.parametrize('raw, want', [
  (None, None), ('', 6.0), ('  \n', 6.0), ('0', 0.0), ('600', 6.0), ('1200', 12.0), (' 450 ', 4.5), ('00000600', 6.0),
  ('1201', None), ('-5', None), ('+5', None), ('6.0', None), ('abc', None), ('1_0', None), ('6 m', None),
  ('�', None), ('9' * 5000, None), ('000000600', None), ('٣', None),
])
def test_parse_marker(raw, want):
  assert parse_marker(raw) == want


@pytest.mark.parametrize('raw_b, want', [
  (None, None), (b'', 6.0), (b'600', 6.0), (b'  \n', 6.0), (b' ' * 64, 6.0),
  (b' ' * 64 + b'x', None), (b'600' + b'x' * 62, None), (b'invalid', None),
])
def test_parse_marker_bytes(raw_b, want):
  assert parse_marker_bytes(raw_b) == want


# --- horizon --------------------------------------------------------------

def path(end=96.0, n=33):
  return [end * i / (n - 1) for i in range(n)]


def test_horizon_from_path():
  assert horizon_from_path(path(96.0)) == pytest.approx(96.0)
  assert horizon_from_path(path(96.0)[:-1]) is None
  bent = path(96.0)
  bent[20] = bent[19] - 0.5
  assert horizon_from_path(bent) is None
  nan = path(96.0)
  nan[5] = math.nan
  assert horizon_from_path(nan) is None
  assert horizon_from_path([-1.0] + path(96.0)[1:]) is None
  assert horizon_from_path(path(600.0)) is None
  assert horizon_from_path(None) is None


# --- commitment and gates -------------------------------------------------

def test_arms_on_commitment_and_brakes_geometrically():
  line = StopLine(6.0)
  a = run(line, 20)
  assert line.active and line.target_m == pytest.approx(90.0 - 19 * 13.0 * DT, abs=1e-6)
  assert -1.25 <= a <= -0.85


def test_experimental_without_stop_commitment_never_arms():
  line = StopLine(6.0)
  assert run(line, 40, stop_committed=False, horizon_m=120.0) is None
  assert not line.active and not line.pending


@pytest.mark.parametrize('block', [
  dict(lead_relevant=True), dict(lead_drel=50.0), dict(v_ego=20.0), dict(v_ego=25.0), dict(experimental=False),
  dict(force_stop=True), dict(pedal=True), dict(standstill=True), dict(horizon_m=None), dict(horizon_m=150.0),
  dict(enabled=False), dict(stop_committed=False),
])
def test_gate_blocks_arming(block):
  line = StopLine(6.0)
  assert run(line, 5, **block) is None
  assert not line.active


def test_blocked_commitment_arms_once_the_gate_opens():
  line = StopLine(6.0)
  assert run(line, 5, lead_drel=50.0) is None
  assert line.pending
  assert run(line, 1) is not None


def test_lead_one_beyond_target_margin_does_not_block():
  line = StopLine(6.0)
  assert run(line, 5, lead_drel=101.0) is not None


def test_lead_mid_approach_dead_reckons_without_reanchoring():
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  before = line.target_m
  assert run(line, 3, v_ego=5.0, lead_relevant=True, lead_drel=5.0, horizon_m=8.0) is None
  assert line.active and line.target_m == pytest.approx(before - 3 * 5.0 * DT)
  assert run(line, 1, v_ego=5.0) is not None
  assert line.target_m == pytest.approx(before - 4 * 5.0 * DT)


def test_lead_owned_long_horizon_does_not_green_clear():
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  before = line.target_m
  long_h = before + sl.GREEN_HORIZON_MARGIN_M + 20.0
  assert run(line, 20, v_ego=5.0, lead_relevant=True, lead_drel=5.0, horizon_m=long_h) is None
  assert line.active and line.pending is False
  assert line.target_m == pytest.approx(before - 20 * 5.0 * DT)
  assert run(line, 1, v_ego=5.0, horizon_m=before) is not None
  assert line.active


def test_slew_references_previous_selected_output():
  line = StopLine(6.0)
  assert run(line, 1, a_prev=0.4) == pytest.approx(0.4 - sl.J_BRAKE * DT)
  assert run(line, 1, a_prev=-2.0) == pytest.approx(-2.0 + sl.J_RELEASE * DT)


def test_positive_first_frame_cannot_remove_model_braking():
  # The hook appends the candidate and keeps every stock candidate: min() still picks the model.
  line = StopLine(6.0)
  cand = run(line, 1, a_prev=0.4)
  assert cand == pytest.approx(0.3)
  assert min(0.5, 0.5, -1.4, cand) == -1.4


def test_relinquished_frame_releases_at_j_release():
  line = StopLine(6.0)
  run(line, 5)
  a_prev = -2.0  # e2e won the last frames
  for _ in range(5):
    cand = run(line, 1, a_prev=a_prev)
    assert cand <= a_prev + sl.J_RELEASE * DT + EPS
    selected = min(-1.49, cand, 0.0)
    assert selected - a_prev <= sl.J_RELEASE * DT + EPS
    a_prev = selected


# --- target bookkeeping ---------------------------------------------------

def test_target_dead_reckons_and_shrinks_only():
  line = StopLine(6.0)
  run(line, 1, horizon_m=120.0)
  assert line.target_m == pytest.approx(114.0)
  run(line, 1, horizon_m=161.0)
  assert line.target_m == pytest.approx(114.0 - 13.0 * DT)
  run(line, 1, horizon_m=103.0)
  assert line.target_m == pytest.approx(97.0)
  run(line, 1, horizon_m=None)
  assert line.target_m == pytest.approx(97.0 - 13.0 * DT)


def test_target_floor_at_arm():
  line = StopLine(6.0)
  run(line, 1, horizon_m=7.0)
  assert line.target_m == pytest.approx(2.0)


# --- drops and re-arming --------------------------------------------------

def test_green_drops_and_stays_dropped_until_a_new_commitment():
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  green = line.target_m + sl.GREEN_HORIZON_MARGIN_M + 5.0
  assert run(line, 9, v_ego=5.0, horizon_m=green) is not None
  assert line.active
  assert run(line, 1, v_ego=5.0, horizon_m=green) is None
  assert not line.active
  assert run(line, 40, v_ego=5.0, horizon_m=green) is None  # identical inputs: no re-arm
  assert run(line, 40, v_ego=5.0, horizon_m=64.0) is None   # nor on a shorter horizon
  assert not line.active and not line.pending
  assert run(line, 2, v_ego=5.0, horizon_m=64.0, stop_committed=False) is None
  assert run(line, 1, v_ego=5.0, horizon_m=64.0) is not None  # fresh commitment edge
  assert line.target_m == pytest.approx(58.0)


def test_model_should_stop_vetoes_green():
  line = StopLine(6.0)
  run(line, 10)
  assert run(line, 30, horizon_m=line.target_m + 50.0, model_should_stop=True) is not None
  assert line.active


def test_experimental_off_dwell_drops_without_rearm():
  line = StopLine(6.0)
  run(line, 10)
  assert run(line, 9, experimental=False) is not None
  assert run(line, 1, experimental=False) is None
  assert run(line, 20) is None
  assert not line.active and not line.pending


def test_standstill_drops_without_rearm():
  line = StopLine(6.0)
  run(line, 10)
  assert run(line, 1, v_ego=0.0, standstill=True) is None
  assert run(line, 20, v_ego=1.0, horizon_m=6.0) is None
  assert not line.active and not line.pending


@pytest.mark.parametrize('clear', [dict(pedal=True), dict(v_ego=22.5), dict(enabled=False), dict(force_stop=True),
                                   dict(stop_committed=False)])
def test_clear_same_frame(clear):
  line = StopLine(6.0)
  run(line, 10)
  assert run(line, 1, **clear) is None
  assert not line.active and not line.pending


def test_reset():
  line = StopLine(6.0)
  run(line, 10)
  line.reset()
  assert not line.active and not line.pending and not line.committed_prev
  assert line.green_s == 0.0 and line.exp_off_s == 0.0


@pytest.mark.parametrize('after', [dict(pedal=True), dict(enabled=False), dict(force_stop=True), dict(v_ego=22.5)])
def test_hard_clear_after_green_does_not_rearm_until_new_commitment(after):
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  green = line.target_m + sl.GREEN_HORIZON_MARGIN_M + 5.0
  assert run(line, 10, v_ego=5.0, horizon_m=green) is None
  assert not line.active
  assert run(line, 2, horizon_m=70.0, **after) is None
  assert run(line, 20, v_ego=5.0, horizon_m=70.0) is None
  assert not line.active and not line.pending
  assert run(line, 2, v_ego=5.0, horizon_m=70.0, stop_committed=False) is None
  assert run(line, 1, v_ego=5.0, horizon_m=70.0) is not None


def test_missing_commitment_evidence_does_not_rearm():
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  green = line.target_m + sl.GREEN_HORIZON_MARGIN_M + 5.0
  assert run(line, 10, v_ego=5.0, horizon_m=green) is None
  assert not line.active
  assert run(line, 2, v_ego=5.0, horizon_m=70.0, stop_committed=None) is None
  assert run(line, 20, v_ego=5.0, horizon_m=70.0) is None
  assert not line.active and not line.pending
  assert run(line, 2, v_ego=5.0, horizon_m=70.0, stop_committed=False) is None
  assert run(line, 1, v_ego=5.0, horizon_m=70.0) is not None


def test_missing_commitment_evidence_keeps_active_stop():
  line = StopLine(6.0)
  run(line, 10, v_ego=5.0)
  before = line.target_m
  assert run(line, 1, v_ego=5.0, stop_committed=None) is None
  assert line.active and line.target_m == pytest.approx(before)
  assert run(line, 1, v_ego=5.0) is not None


def test_hard_clear_after_standstill_does_not_rearm_until_new_commitment():
  line = StopLine(6.0)
  run(line, 10)
  assert run(line, 1, v_ego=0.0, standstill=True) is None
  assert run(line, 2, v_ego=5.0, pedal=True, horizon_m=70.0) is None
  assert run(line, 20, v_ego=5.0, horizon_m=70.0) is None
  assert not line.active
  assert run(line, 2, v_ego=5.0, horizon_m=70.0, stop_committed=False) is None
  assert run(line, 1, v_ego=5.0, horizon_m=70.0) is not None


# --- profiles -------------------------------------------------------------

def test_law_13mps_96m_offset_6():
  assert StopLine.law(13.0, 90.0, 0.0) == pytest.approx(-1.02, abs=0.03)
  assert StopLine.law(13.0, 90.0, -0.95) == pytest.approx(-0.99, abs=0.03)
  assert StopLine.law(2.5, sl.D_SOFT_M + sl.STOP_MARGIN_M, -sl.A_SOFT) == pytest.approx(-sl.A_SOFT, abs=0.05)
  assert StopLine.law(0.2, 2.0, -0.3) == -sl.A_FINISH_MIN
  assert StopLine.law(0.2, 0.6, -0.3) == -sl.A_SOFT
  assert StopLine.law(4.0, 3.0, -1.5) == -sl.A_FINISH_MAX
  assert StopLine.law(8.0, 5.0, -1.0) == -sl.A_FIRM_MAX
  assert StopLine.law(13.0, 40.0, 0.0) == -sl.A_FIRM_MAX
  assert -0.2 < StopLine.law(2.0, 30.0, -0.1) < 0.0


def test_approach_13mps_96m():
  r = simulate(13.0 / KMH, 96.0)
  assert 0.0 <= r['short_m'] <= 3.0
  assert 5.5 <= r['soft_at_m'] <= 9.0
  assert r['min_fast'] >= -1.35
  assert r['last8_mean'] >= -0.7


MATRIX = [(v, h) for v in (30, 50, 60) for h in (50, 90, 130)]


@pytest.mark.parametrize('v0_kmh, horizon_m', MATRIX)
def test_simulated_approach(v0_kmh, horizon_m):
  r = simulate(v0_kmh, horizon_m)
  print(f"{v0_kmh} km/h {horizon_m} m: short {r['short_m']:.2f} m, min>15km/h {r['min_fast']:.2f}, "
        f"last8 {r['last8_mean']:.2f}, soft at {r['soft_at_m']:.1f} m, need {required_decel(v0_kmh, horizon_m):.2f}")
  assert r['min_fast'] >= -sl.A_FIRM_MAX
  if required_decel(v0_kmh, horizon_m) <= sl.A_FIRM_MAX:
    assert 0.0 <= r['short_m'] <= 3.0
    assert r['last8_mean'] >= -0.7


@pytest.mark.parametrize('v0_kmh, horizon_m', MATRIX)
def test_arbitrated_approach_never_worse_than_stock(v0_kmh, horizon_m):
  stock = simulate(v0_kmh, horizon_m, candidate=False, model=late_model)
  both = simulate(v0_kmh, horizon_m, candidate=True, model=late_model)
  print(f"{v0_kmh} km/h {horizon_m} m: stock short {stock['short_m']:.2f} m, with candidate {both['short_m']:.2f} m")
  assert both['short_m'] >= stock['short_m'] - EPS
  assert both['short_m'] > -6.0  # the model's own brake stays in: still short of the model endpoint
  if required_decel(v0_kmh, horizon_m) <= sl.A_FIRM_MAX:
    assert 0.0 <= both['short_m'] <= 3.0
    assert both['last8_mean'] >= -0.7


def test_early_model_should_stop_ends_short_via_stopping_ramp():
  r = simulate(50.0, 90.0, model=late_model, model_stop_below=2.0)
  assert r['short_m'] >= 0.0


@pytest.mark.parametrize('lag_s', [0.15, 0.3])
def test_approach_lag_bounds(lag_s):
  r = simulate(50.0, 90.0, lag_s=lag_s)
  assert 0.0 <= r['short_m'] <= 3.0
  assert r['last8_mean'] >= -0.7


def test_offset_moves_the_stop():
  assert simulate(50.0, 90.0, offset=0.0)['short_m'] >= 0.0
  assert simulate(50.0, 90.0, offset=12.0)['short_m'] >= 0.0
'''

OLD_IMPORT = "from openpilot.starpilot.longitudinal.force_stop import StopPlan\n"
NEW_IMPORT = (
    "from openpilot.starpilot.longitudinal.force_stop import StopPlan\n"
    "from openpilot.starpilot.longitudinal.stop_line import StopLine, horizon_from_path, parse_marker_bytes as parse_stop_line_marker_bytes\n"
)

OLD_MARKER = "CRUISE_SHAPER = _read_cruise_shaper()\n"
NEW_MARKER = """CRUISE_SHAPER = _read_cruise_shaper()

# /data/ops_patches/camry_stop_line (read once at import): once Conditional
# Experimental Mode has committed to a red light with no lead, a geometric
# stop-line candidate (StopLine) joins the min() and brakes steadily toward
# the model stop horizon minus this offset; every stock candidate stays in.
# Absent = stock; empty = 6.0 m; integer cm 0..1200; unreadable or bad
# parse = stock with a log line.
STOP_LINE_MARKER = "/data/ops_patches/camry_stop_line"


def _read_stop_line():
  if not os.path.isfile(STOP_LINE_MARKER):
    return None
  try:
    with open(STOP_LINE_MARKER, "rb") as f:
      raw_b = f.read(65)
    offset = parse_stop_line_marker_bytes(raw_b)
  except (OSError, ValueError, UnicodeError) as e:
    cloudlog.warning("camry_stop_line: marker read failed (%s); stock", e)
    return None
  if offset is None:
    cloudlog.warning("camry_stop_line: marker parse failed (%r); stock", raw_b)
  else:
    cloudlog.info("camry_stop_line: offset %.2f m", offset)
  return offset


STOP_LINE_OFFSET_M = _read_stop_line()
"""

OLD_INIT = (
    "    self.force_stop_plan = StopPlan()\n"
    "    self.follow_jerk = FollowJerk(CP)\n"
)
NEW_INIT = (
    "    self.force_stop_plan = StopPlan()\n"
    "    self.stop_line = StopLine(STOP_LINE_OFFSET_M) if STOP_LINE_OFFSET_M is not None else None\n"
    "    self.follow_jerk = FollowJerk(CP)\n"
)

OLD_CANDIDATES = (
    "    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),\n"
    "                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]\n"
    "    if sm['selfdriveState'].experimentalMode:\n"
    "      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))\n"
)
NEW_CANDIDATES = (
    "    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),\n"
    "                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]\n"
    "    if self.stop_line is not None:\n"
    "      lead_one = sm['radarState'].leadOne\n"
    "      stop_line_accel = self.stop_line.step(\n"
    "        self.dt, v_ego, horizon_from_path(sm['modelV2'].position.x),\n"
    "        stop_committed=(None if wheel_coast is None else bool(getattr(wheel_coast[0], 'conditional_stop', False) is True)),\n"
    "        experimental=bool(sm['selfdriveState'].experimentalMode), enabled=not reset_state,\n"
    "        lead_relevant=bool(relevant_lead), lead_drel=float(lead_one.dRel) if lead_one.present else None,\n"
    "        force_stop=bool(self.force_stop_plan.forcing or self.force_stop_plan.obstacle_m is not None),\n"
    "        pedal=bool(sm['carState'].gasPressed or sm['carState'].brakePressed),\n"
    "        standstill=bool(sm['carState'].standstill), model_should_stop=bool(output_should_stop_e2e),\n"
    "        a_prev=float(a_prev))\n"
    "      if stop_line_accel is not None:\n"
    "        # One more candidate for the min(): a steady geometric brake toward the\n"
    "        # model stop horizon minus the marker offset. Every stock candidate stays\n"
    "        # in, so the output is never softer than stock.\n"
    "        candidates.append((stop_line_accel, LongitudinalPlanSource.cruise, False))\n"
    "    if sm['selfdriveState'].experimentalMode:\n"
    "      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))\n"
)

EDITS = [(OLD_IMPORT, NEW_IMPORT), (OLD_MARKER, NEW_MARKER), (OLD_INIT, NEW_INIT), (OLD_CANDIDATES, NEW_CANDIDATES)]


ROOT = "/data/openpilot"


def _confined(path):
  root = os.path.realpath(ROOT)
  abs_path = path if os.path.isabs(path) else os.path.abspath(path)
  if os.path.lexists(abs_path):
    real = os.path.realpath(abs_path)
  else:
    real = os.path.join(os.path.realpath(os.path.dirname(abs_path) or "."), os.path.basename(abs_path))
  return real == root or real.startswith(root + os.sep)


def write_if_changed(path, text):
  if not _confined(path):
    print(f"ERROR: {path} escapes {ROOT}")
    raise SystemExit(1)
  if os.path.isfile(path):
    with open(path) as f:
      if f.read() == text:
        return False
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "w") as f:
    f.write(text)
  return True


def main():
  if not os.path.isfile(PLANNER_PATH):
    print(f"ERROR: {PLANNER_PATH} missing")
    return 1
  if not _confined(PLANNER_PATH):
    print(f"ERROR: {PLANNER_PATH} escapes {ROOT}")
    return 1
  with open(PLANNER_PATH) as f:
    content = f.read()

  todo = []
  for old, new in EDITS:
    if new in content:
      if content.count(new) != 1:
        print(f"ERROR: already-applied edit found {content.count(new)}x (expected 1) in longitudinal_planner.py; patch not applied")
        print(f"  anchor: {new.splitlines()[0]!r}")
        return 1
      continue
    if content.count(old) != 1:
      print(f"ERROR: anchor found {content.count(old)}x (expected 1) in longitudinal_planner.py; patch not applied")
      print(f"  anchor: {old.splitlines()[0]!r}")
      return 1
    todo.append((old, new))

  written = [p for p, text in ((MODULE_PATH, MODULE), (TEST_PATH, TEST)) if write_if_changed(p, text)]
  if not os.path.isfile(TEST_INIT_PATH):
    write_if_changed(TEST_INIT_PATH, '"""Native planner input tests."""\n')
    written.append(TEST_INIT_PATH)

  if not todo and not written:
    print("camry stop line already applied")
    return 0

  for old, new in todo:
    content = content.replace(old, new, 1)
  if todo:
    with open(PLANNER_PATH, "w") as f:
      f.write(content)
  print(f"patched longitudinal_planner.py ({len(todo)} edits); wrote {len(written)} files")
  return 0


if __name__ == "__main__":
  sys.exit(main())
