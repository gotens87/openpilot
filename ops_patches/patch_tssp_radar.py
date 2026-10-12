#!/usr/bin/env python3
# /data/ops_patches/patch_tssp_radar.py
# TSS-P Camry: parse the Continental radar instead of running vision-only ACC.
#
# values.py: restore the radar DBC on the Camry platform.
# interface.py: only expect radar traffic when the filter hardware is present;
#   a stock Camry keeps radarUnavailable so a silent bus can't fault.
# radar_interface.py: the TSS-P Continental radar publishes one cluster
#   message (0x680-0x685) per track and has no STATUS_MSG, so it gets its own
#   path next to the Denso one. Ego speed comes from the PT bus wheel speeds.
#   LAT_DIST keeps its raw sign: on this car the negated form mirrors points
#   left/right, confirmed on-road. The Denso path is untouched.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

# patch_sdsu_long.py must run first: the radarUnavailable gate uses camry_sdsu
DEPENDENCIES = {"interface.py": "camry_sdsu = candidate == CAR.TOYOTA_CAMRY"}

EDITS = {
  "values.py": [
    ("""    CarSpecs(mass=3400. * CV.LB_TO_KG, wheelbase=2.82448, steerRatio=13.7, tireStiffnessFactor=0.7933),
    {Bus.pt: 'toyota_nodsu_pt_generated'},""",
     """    CarSpecs(mass=3400. * CV.LB_TO_KG, wheelbase=2.82448, steerRatio=13.7, tireStiffnessFactor=0.7933),
    dbc_dict('toyota_nodsu_pt_generated', 'toyota_radar_dsu_tssp'),"""),
  ],
  "interface.py": [
    ("""    ret.radarUnavailable = Bus.radar not in DBC[candidate]""",
     """    ret.radarUnavailable = Bus.radar not in DBC[candidate] or (candidate == CAR.TOYOTA_CAMRY and not camry_sdsu)"""),
  ],
  "radar_interface.py": [
    ("""from opendbc.car.toyota.values import DBC, ToyotaFlags""",
     """from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags"""),
    ("""from opendbc.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(CP):""",
     """from opendbc.car.interfaces import RadarInterfaceBase

RADAR_ACC_TSSP_CAR = {CAR.TOYOTA_CAMRY}
TSSP_CLUSTER_MSGS = list(range(0x680, 0x686))
KPH_TO_MS = 1. / 3.6
TSSP_RADAR_EGO_SPEED_SCALE = 0.922


def _create_radar_can_parser(CP):"""),
    ("""  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)""",
     """  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


def _create_tssp_radar_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(addr, 10) for addr in TSSP_CLUSTER_MSGS], 1)


def _create_wheel_speed_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.pt], [("WHEEL_SPEEDS", 80)], 0)"""),
    ("""class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    if CP.flags & ToyotaFlags.TSS2:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()""",
     """class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_acc_tssp = CP.carFingerprint in RADAR_ACC_TSSP_CAR

    if self.radar_acc_tssp:
      self.RADAR_MSGS = TSSP_CLUSTER_MSGS
      self.rcp = None if CP.radarUnavailable else _create_tssp_radar_can_parser(CP)
      self.pt_cp = None if CP.radarUnavailable else _create_wheel_speed_can_parser(CP)
      self.trigger_msg = self.RADAR_MSGS[-1]
    else:
      if CP.flags & ToyotaFlags.TSS2:
        self.RADAR_A_MSGS = list(range(0x180, 0x190))
        self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
      else:
        self.RADAR_A_MSGS = list(range(0x210, 0x220))
        self.RADAR_B_MSGS = list(range(0x220, 0x230))

      self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

      self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
      self.pt_cp = None
      self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()"""),
    ("""    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)""",
     """    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.pt_cp is not None:
      self.pt_cp.update(can_strings)

    if self.trigger_msg not in self.updated_messages:
      return None

    if self.pt_cp is not None and not self.pt_cp.can_valid:
      self.updated_messages.clear()
      ret = RadarData()
      ret.errors.canError = True
      return ret

    rr = self._update(self.updated_messages)"""),
    ("""  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.rcp.vl['STATUS_MSG']['RADAR_STATUS'] != 1 or self.rcp.vl['STATUS_MSG']['RADAR_PRE_FAULT'] != 0:
      ret.errors.radarUnavailableTemporary = True""",
     """  def _get_v_ego(self):
    ws = self.pt_cp.vl["WHEEL_SPEEDS"]
    wheel_speed = (ws["WHEEL_SPEED_FL"] + ws["WHEEL_SPEED_FR"] +
                   ws["WHEEL_SPEED_RL"] + ws["WHEEL_SPEED_RR"]) / 4.
    return wheel_speed * KPH_TO_MS * self.CP.wheelSpeedFactor

  def _update_tssp(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    v_ego = self._get_v_ego()
    updated_ids = set()
    for ii in sorted(updated_messages):
      if ii not in self.RADAR_MSGS:
        continue

      cpt = self.rcp.vl[ii]
      track_id = int(cpt["ID"])
      if track_id == 0x3f or cpt["LONG_DIST"] <= 0:
        continue

      updated_ids.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = RadarData.RadarPoint()
        self.pts[track_id].trackId = self.track_id
        self.track_id += 1

      self.pts[track_id].dRel = float(cpt["LONG_DIST"])
      self.pts[track_id].yRel = float(cpt["LAT_DIST"])
      self.pts[track_id].vRel = float(cpt["SPEED"]) - v_ego * TSSP_RADAR_EGO_SPEED_SCALE

    for track_id in list(self.pts):
      if track_id not in updated_ids:
        del self.pts[track_id]

    ret.points = list(self.pts.values())
    return ret

  def _update(self, updated_messages):
    if self.radar_acc_tssp:
      return self._update_tssp(updated_messages)

    return self._update_denso(updated_messages)

  def _update_denso(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.rcp.vl['STATUS_MSG']['RADAR_STATUS'] != 1 or self.rcp.vl['STATUS_MSG']['RADAR_PRE_FAULT'] != 0:
      ret.errors.radarUnavailableTemporary = True"""),
  ],
}


def main():
  contents = {}
  for name in set(EDITS) | set(DEPENDENCIES):
    path = os.path.join(TOYOTA, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

  for name, marker in DEPENDENCIES.items():
    if marker not in contents[name]:
      print(f"ERROR: {name} is missing {marker!r}; run patch_sdsu_long.py first")
      return 1

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
    print("tssp radar already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TOYOTA, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())
