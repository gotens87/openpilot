#!/usr/bin/env python3
# Continental radar support for TSS-P NO_DSU Toyotas (2018 Camry). The RADAR_ACC_TSSP path
# reads the radar's pre-clustered CLUSTER_F..R_A output (0x680-0x685) and derives vRel from
# WHEEL_SPEEDS, since the Continental radar reports ABSOLUTE target speed (unlike Denso/TSS2,
# which report relative). Ported from Smartype/openpilot:toyota_radar_dsu_tssp_v_ego.
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.toyota.values import DBC, TSS2_CAR, CAR
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase


# Continental radar on TSS-P NO_DSU cars (2018 Camry). Uses the radar's pre-clustered
# output (CLUSTER_F..R_A, 0x680-0x685) rather than the raw OBJECT_0..11 list.
RADAR_ACC_TSSP_CAR = {CAR.TOYOTA_CAMRY}
CLUSTER_MSGS = list(range(0x680, 0x686))

KPH_TO_MS = 1. / 3.6
# NOTE: this empirical scale comes directly from the StarPilot follow-up patch. Its physical
# basis (e.g. radar speed reference vs. vehicle speed reference) has not been independently
# verified and should be validated on the target car before trusting lead-tracker output.
RADAR_EGO_REF_SCALE = 0.922


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

  return CANParser(DBC[car_fingerprint]['radar'], messages, 1)


def _create_radar_acc_tssp_can_parser(car_fingerprint):
  # Continental cluster messages on bus 1, ~10 Hz
  messages = [(addr, 10) for addr in CLUSTER_MSGS]
  return CANParser(DBC[car_fingerprint]['radar'], messages, 1)


def _create_pt_speed_parser(car_fingerprint):
  # The Continental radar reports ABSOLUTE target speed; we need ego speed to derive vRel.
  return CANParser(DBC[car_fingerprint]['pt'], [("WHEEL_SPEEDS", 80)], 0)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP                    # required for _get_v_ego(); base class does not store it
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep
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

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)
    if self.pt_cp is not None:
      self.pt_cp.update_strings(can_strings)

    if self.trigger_msg not in self.updated_messages:
      return None

    # The Continental radar reports ABSOLUTE speed, so vRel is meaningless without a live
    # WHEEL_SPEEDS feed. If bus 0 drops out, surface a canError so radard clears its tracks
    # and falls back to vision rather than leaving stale radar tracks in place.
    if self.pt_cp is not None and not self.pt_cp.can_valid:
      self.updated_messages.clear()
      ret = car.RadarData.new_message()
      ret.errors = ["canError"]
      return ret

    rr = self._update_radar_acc_tssp(self.updated_messages) if self.radar_acc_tssp \
        else self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _get_v_ego(self):
    # The Continental radar reports ABSOLUTE target speed, so vRel needs ego speed.
    # Mirrors CarStateBase.parse_wheel_speeds: mean of the 4 wheel speeds (km/h -> m/s)
    # with the per-car wheelSpeedFactor correction.
    ws = self.pt_cp.vl["WHEEL_SPEEDS"]
    return (ws["WHEEL_SPEED_FL"] + ws["WHEEL_SPEED_FR"] +
            ws["WHEEL_SPEED_RL"] + ws["WHEEL_SPEED_RR"]) / 4. * KPH_TO_MS * self.CP.wheelSpeedFactor

  def _update_radar_acc_tssp(self, updated_messages):
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    v_ego = self._get_v_ego()
    updated_ids = set()
    for ii in sorted(updated_messages):
      if ii in self.RADAR_MSGS:
        cpt = self.rcp.vl[ii]
        track_id = int(cpt['ID'])
        if track_id != 0x3f and cpt['LONG_DIST'] > 0:   # 0x3f = "no object"
          updated_ids.add(track_id)
          if track_id not in self.pts:
            self.pts[track_id] = car.RadarData.RadarPoint.new_message()
            self.pts[track_id].trackId = self.track_id
            self.track_id += 1
          self.pts[track_id].dRel = float(cpt['LONG_DIST'])      # m, from front of car
          # NOTE: sign flipped relative to raw LAT_DIST based on the StarPilot follow-up
          # patch; verify on the actual car that positive yRel means left.
          self.pts[track_id].yRel = -float(cpt['LAT_DIST'])
          # Continental radar reports absolute speed; subtract ego speed (with empirically
          # scaled reference) to get relative speed.
          self.pts[track_id].vRel = float(cpt['SPEED']) - v_ego * RADAR_EGO_REF_SCALE
          self.pts[track_id].aRel = float('nan')
          self.pts[track_id].yvRel = float(cpt['LAT_SPEED'])
          self.pts[track_id].measured = True

    # Immediate delete when a cluster ID disappears. radard rebuilds its tracks from the
    # ar_pts list each frame and ignores RadarPoint.measured, so a TTL/coast is not useful.
    for track_id in list(self.pts):
      if track_id not in updated_ids:
        del self.pts[track_id]

    ret.points = list(self.pts.values())
    return ret

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
