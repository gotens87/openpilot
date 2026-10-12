#!/usr/bin/env python3
# /data/ops_patches/patch_camry_sdsu_tests.py
# Regression tests for the TSS-P Camry sDSU port. Installs a new test file;
# fails if the port patchers haven't run, so tests can't land without the code.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"
TEST_PATH = os.path.join(TOYOTA, "tests", "test_camry_sdsu.py")

DEPENDENCIES = {
  "interface.py": "camry_sdsu = candidate == CAR.TOYOTA_CAMRY",
  "radar_interface.py": "def _update_tssp(self, updated_messages):",
  "values.py": "def camry_hybrid_tune(CP) -> bool:",
}

TEST_FILE = r'''import unittest

import numpy as np

from opendbc.can import CANParser, CANPacker
from opendbc.car import Bus, structs
from opendbc.car.structs import CarParams
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags, CarControllerParams, camry_hybrid_tune, stopping_decel_rate

Ecu = CarParams.Ecu

SDSU_FINGERPRINT = {0: {0x2FF: 4}, 1: {}, 2: {}}
RADAR_FILTER_FINGERPRINT = {0: {0x2AA: 8}, 1: {}, 2: {}}
EMPTY_FINGERPRINT = {0: {}, 1: {}, 2: {}}
HYBRID_FW = [CarParams.CarFw.new_message(ecu=Ecu.hybrid, fwVersion=b"\x0281E00600\x00\x00\x00\x00\x00\x00\x00")]


def get_camry(fp):
  return CarInterface.get_params(CAR.TOYOTA_CAMRY, fp, HYBRID_FW, False, False, False)


class FakeCarState:
  def __init__(self):
    self.out = structs.CarState()
    self.gvc = 0.0
    self.acc_type = 1
    self.pcm_acc_status = 0
    self.pcm_follow_distance = 0
    self.lkas_hud = {}
    self.pre_collision_2 = {}
    self.secoc_synchronization = None


class TestCamrySdsuParams(unittest.TestCase):
  def test_sdsu_enables_openpilot_longitudinal(self):
    for fp in (SDSU_FINGERPRINT, RADAR_FILTER_FINGERPRINT):
      with self.subTest(fp=fp):
        cp = get_camry(fp)
        self.assertTrue(cp.openpilotLongitudinalControl)
        # EPS factor 73 | LONG_FILTER: panda selects the filter TX set on this exact value
        self.assertEqual(cp.safetyConfigs[0].safetyParam, 4169)
        self.assertTrue(cp.flags & ToyotaFlags.LONG_FILTER)
        self.assertFalse(cp.radarUnavailable)
        self.assertFalse(cp.autoResumeSng)
        self.assertAlmostEqual(cp.minEnableSpeed, -1.0)
        self.assertAlmostEqual(cp.longitudinalActuatorDelay, 0.05)

  def test_stock_camry_unchanged(self):
    cp = get_camry(EMPTY_FINGERPRINT)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 73 | (2 << 8))
    self.assertFalse(cp.flags & ToyotaFlags.LONG_FILTER)
    # radar traffic is only expected with the filter hardware present
    self.assertTrue(cp.radarUnavailable)

  def test_tune_flag_defaults_off(self):
    cp = get_camry(SDSU_FINGERPRINT)
    self.assertFalse(camry_hybrid_tune(cp))
    self.assertEqual(CarControllerParams(cp).ACCEL_MAX, 2.0)
    self.assertIsNone(stopping_decel_rate(cp))
    controller = CarController({Bus.pt: DBC[cp.carFingerprint][Bus.pt]}, cp)
    self.assertFalse(controller.camry_hybrid_tune)


class TestCamryStandstillLatch(unittest.TestCase):
  def setUp(self):
    self.cp = get_camry(SDSU_FINGERPRINT)
    self.controller = CarController({Bus.pt: DBC[self.cp.carFingerprint][Bus.pt]}, self.cp)
    self.state = FakeCarState()
    self.parser = CANParser(DBC[self.cp.carFingerprint][Bus.pt], [("ACC_CONTROL", 33)], 0)
    self.now = 1_000_000_000

  def step(self, standstill, pcm_acc_status, resume=False):
    self.state.out.standstill = standstill
    self.state.pcm_acc_status = pcm_acc_status
    command = structs.CarControl()
    command.cruiseControl.resume = resume
    command = command.as_reader()
    release = None
    for _ in range(4):
      _, sends = self.controller.update(command, self.state, self.now)
      self.now += 10_000_000
      for frame in sends:
        if frame[0] == 0x343:
          self.parser.update([[self.now, [frame]]])
          release = self.parser.vl["ACC_CONTROL"]["RELEASE_STANDSTILL"]
    self.assertIsNotNone(release)
    return release

  def test_standstill_latch(self):
    self.assertEqual(self.step(False, 8), 1)  # driving: released
    self.assertEqual(self.step(True, 8), 0)   # stop edge: request the hold
    self.assertEqual(self.step(True, 8), 0)   # still waiting on the PCM
    self.assertEqual(self.step(True, 7), 1)   # PCM latched the stop: release
    self.assertEqual(self.step(True, 8, resume=True), 1)  # resume releases


class TestCamryTsspRadar(unittest.TestCase):
  def test_radar_dbc_restored(self):
    self.assertEqual(DBC[CAR.TOYOTA_CAMRY][Bus.radar], "toyota_radar_dsu_tssp")

  def test_stock_camry_has_no_radar_parser(self):
    self.assertIsNone(RadarInterface(get_camry(EMPTY_FINGERPRINT)).rcp)

  def test_tssp_points_keep_raw_lat_sign(self):
    cp = get_camry(SDSU_FINGERPRINT)
    ri = RadarInterface(cp)
    self.assertIsNotNone(ri.rcp)

    pt_packer = CANPacker(DBC[CAR.TOYOTA_CAMRY][Bus.pt])
    radar_packer = CANPacker(DBC[CAR.TOYOTA_CAMRY][Bus.radar])
    frames = [pt_packer.make_can_msg("WHEEL_SPEEDS", 0, {"WHEEL_SPEED_FL": 0., "WHEEL_SPEED_FR": 0.,
                                                         "WHEEL_SPEED_RL": 0., "WHEEL_SPEED_RR": 0.})]
    for addr in range(0x680, 0x686):
      values = {"ID": 0x3F, "LONG_DIST": 0., "LAT_DIST": 0., "SPEED": 0., "LAT_SPEED": 0.}
      if addr == 0x681:
        values = {"ID": 5, "LONG_DIST": 80., "LAT_DIST": -4., "SPEED": 20., "LAT_SPEED": 1.}
      frames.append(radar_packer.make_can_msg(addr, 1, values))

    rr = ri.update([[1_000_000_000, frames]])
    self.assertIsNotNone(rr)
    self.assertFalse(rr.errors.canError)
    self.assertEqual(len(rr.points), 1)

    point = rr.points[0]
    lat_dist = ri.rcp.vl[0x681]["LAT_DIST"]
    long_dist = ri.rcp.vl[0x681]["LONG_DIST"]
    self.assertLess(lat_dist, 0.)
    # RadarPoint fields are Float32, so the float64 parser values are rounded
    # on assignment (~1e-7 relative, ~7.6e-6 at 80 m); compare against the
    # float32-rounded expectations instead of the raw float64 values
    self.assertEqual(point.yRel, float(np.float32(lat_dist)))  # a right-side target stays on the right
    self.assertEqual(point.dRel, float(np.float32(long_dist)))


if __name__ == "__main__":
  unittest.main()
'''


def main():
  if not os.path.isdir(os.path.dirname(TEST_PATH)):
    print(f"ERROR: {os.path.dirname(TEST_PATH)} missing")
    return 1

  for name, marker in DEPENDENCIES.items():
    path = os.path.join(TOYOTA, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      if marker not in f.read():
        print(f"ERROR: {name} is missing {marker!r}; run the port patchers first")
        return 1

  if os.path.isfile(TEST_PATH):
    with open(TEST_PATH) as f:
      if f.read() == TEST_FILE:
        print("camry sdsu tests already installed")
        return 0

  with open(TEST_PATH, "w") as f:
    f.write(TEST_FILE)
  print("installed tests/test_camry_sdsu.py")
  return 0


if __name__ == "__main__":
  sys.exit(main())
