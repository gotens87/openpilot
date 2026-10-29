"""Stop-line candidate: marker parse, commitment gating, shrink-only target, profiles and arbitrated approaches."""
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
