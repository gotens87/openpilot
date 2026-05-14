#!/usr/bin/env python3
# Continental radar support for TSS-P NO_DSU Toyotas (2018 Camry). The RADAR_ACC_TSSP path
# reads the radar's pre-clustered CLUSTER_F..R_A output (0x680-0x685) and derives vRel from
# WHEEL_SPEEDS, since the Continental radar reports ABSOLUTE target speed (unlike Denso/TSS2,
# which report relative). Ported from Smartype/openpilot:toyota_radar_dsu_tssp_v_ego.
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, TSS2_CAR, CAR
from opendbc.car.interfaces import RadarInterfaceBase

# Continental radar on TSS-P NO_DSU cars (2018 Camry/CHR). Uses the radar's pre-clustered
# output (CLUSTER_F..R_A, 0x680-0x685) rather than the raw OBJECT_0..11 list.
RADAR_ACC_TSSP_CAR = {CAR.TOYOTA_CAMRY}
CLUSTER_MSGS = list(range(0x680, 0x686))

KPH_TO_MS = 1. / 3.6


def _create_radar_can_parser(car_fingerprint):
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))

  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 1)


def _create_radar_acc_tssp_can_parser(car_fingerprint):
  # Continental cluster messages on bus 1, ~10 Hz
  messages = [(addr, 10) for addr in CLUSTER_MSGS]
  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 1)


def _create_pt_speed_parser(car_fingerprint):
  # the Continental radar reports ABSOLUTE target speed; we need ego speed to derive vRel.
  # VERIFIED 2026-05-14: WHEEL_SPEEDS msg + WHEEL_SPEED_{FL,FR,RL,RR} signals (km/h) match
  # toyota/carstate.py; Bus.pt resolves to bus 0 (carstate.py:273). Declared freq 80 Hz is
  # the canonical openpilot value; measured ~83 Hz on this car — within the 10x alive margin.
  return CANParser(DBC[car_fingerprint][Bus.pt], [("WHEEL_SPEEDS", 80)], 0)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0
    self.radar_acc_tssp = CP.carFingerprint in RADAR_ACC_TSSP_CAR

    if self.radar_acc_tssp:
      self.RADAR_MSGS = CLUSTER_MSGS
      self.rcp = None if CP.radarUnavailable else _create_radar_acc_tssp_can_parser(CP.carFingerprint)
      self.pt_cp = None if CP.radarUnavailable else _create_pt_speed_parser(CP.carFingerprint)
      self.trigger_msg = self.RADAR_MSGS[-1]
    else:
      if CP.carFingerprint in TSS2_CAR:
        self.RADAR_A_MSGS = list(range(0x180, 0x190))
        self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
      else:
        self.RADAR_A_MSGS = list(range(0x210, 0x220))
        self.RADAR_B_MSGS = list(range(0x220, 0x230))
      self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}
      self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
      self.pt_cp = None
      self.trigger_msg = self.RADAR_B_MSGS[-1]

    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.pt_cp is not None:
      self.pt_cp.update(can_strings)   # keep ego-speed parser fresh; don't add to triggers

    if self.trigger_msg not in self.updated_messages:
      return None

    # the radar reports ABSOLUTE speed, so vRel is meaningless without a live WHEEL_SPEEDS
    # feed. if the ego-speed parser isn't valid (startup, bus-0 dropout), surface an explicit
    # canError with no points — the same signal _update_radar_acc_tssp raises for a dead
    # radar bus — so radard clears its tracks and fails over to vision. returning None here
    # would read to radard as "trigger not seen yet" and let stale radar tracks linger.
    if self.pt_cp is not None and not self.pt_cp.can_valid:
      self.updated_messages.clear()
      ret = RadarData()
      ret.errors.canError = True
      return ret

    rr = self._update_radar_acc_tssp(self.updated_messages) if self.radar_acc_tssp \
        else self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _get_v_ego(self):
    # the Continental radar reports ABSOLUTE target speed, so vRel needs ego speed.
    # mirrors CarStateBase.parse_wheel_speeds (opendbc/car/interfaces.py:419): mean of the
    # 4 wheel speeds (km/h -> m/s) with the per-car wheelSpeedFactor correction. NO extra
    # smoothing — wheel speeds are low-noise, and any filter here lags v_ego under accel/
    # decel and skews vRel (round-2 review: both reviewers said drop the EMA).
    ws = self.pt_cp.vl["WHEEL_SPEEDS"]
    return (ws["WHEEL_SPEED_FL"] + ws["WHEEL_SPEED_FR"] +
            ws["WHEEL_SPEED_RL"] + ws["WHEEL_SPEED_RR"]) / 4. * KPH_TO_MS * self.CP.wheelSpeedFactor

  def _update_radar_acc_tssp(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    v_ego = self._get_v_ego()
    updated_ids = set()
    for ii in sorted(updated_messages):
      if ii in self.RADAR_MSGS:
        cpt = self.rcp.vl[ii]
        track_id = int(cpt['ID'])
        if track_id != 0x3f and cpt['LONG_DIST'] > 0:   # 0x3f = "no object"
          updated_ids.add(track_id)
          if track_id not in self.pts:
            self.pts[track_id] = RadarData.RadarPoint()
            self.pts[track_id].trackId = self.track_id
            self.track_id += 1
          self.pts[track_id].dRel = float(cpt['LONG_DIST'])      # m, from front of car
          self.pts[track_id].yRel = float(cpt['LAT_DIST'])       # m, LAT_DIST as-is — sign unverified
          self.pts[track_id].vRel = float(cpt['SPEED']) - v_ego  # radar SPEED is absolute
          self.pts[track_id].aRel = float('nan')
          self.pts[track_id].yvRel = float(cpt['LAT_SPEED'])
          self.pts[track_id].measured = True

    # immediate delete (Smartype reference behavior). a TTL/coast here can't help: radard
    # rebuilds its per-track Kalman from ar_pts presence each frame and ignores
    # RadarPoint.measured entirely — and the cluster IDs measured ~99.85% stable anyway.
    for track_id in list(self.pts):
      if track_id not in updated_ids:
        del self.pts[track_id]

    ret.points = list(self.pts.values())
    return ret

  def _update(self, updated_messages):
    # Unchanged stock path: TSS2 (0x180-0x19F) and Denso TSS-P (0x210-0x22F).
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii + 16]['SCORE']

        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = RadarData.RadarPoint()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']
          self.pts[ii].yRel = -cpt['LAT_DIST']
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
