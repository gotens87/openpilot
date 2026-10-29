"""Geometric stop-line braking candidate for a committed red-light stop.

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
