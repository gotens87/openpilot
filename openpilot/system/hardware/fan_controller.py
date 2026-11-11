#!/usr/bin/env python3
import os
import numpy as np

from openpilot.common.pid import PIDController
from openpilot.common.hardware import HARDWARE

# raise fan setpoint on tici/tizi to reduce noise
# after raising LMH threshold in AGNOS 18.1 to prevent CPU throttling
OFFSET = 0 if HARDWARE.get_device_type() == "mici" else 5


# /data/ops_patches/fan_floor (read once at import) raises the ignition-on
# neg_limit floor for a degraded fan that stalls at the stock 30% duty:
# int(contents) clamped to 30..100, empty/unparseable = 50, absent = None
# = stock 30. pos_limit, the ignition-off limits, setpoint and feedforward
# are untouched.
FAN_FLOOR_MARKER = "/data/ops_patches/fan_floor"


def _read_fan_floor():
  if not os.path.isfile(FAN_FLOOR_MARKER):
    return None
  try:
    with open(FAN_FLOOR_MARKER) as f:
      return min(100, max(30, int(f.read())))
  except (OSError, ValueError):
    return 50


FAN_FLOOR = _read_fan_floor()


class FanController:
  def __init__(self, rate: int) -> None:
    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, rate=rate)

  def update(self, cur_temp: float, ignition: bool) -> int:
    self.controller.pos_limit = 100 if ignition else 30
    self.controller.neg_limit = (FAN_FLOOR or 30) if ignition else 0

    if ignition != self.last_ignition:
      self.controller.reset()
    self.last_ignition = ignition

    return int(self.controller.update(
                 error=(cur_temp - (75 + OFFSET)),  # temperature setpoint in C
                 feedforward=np.interp(cur_temp, [60.0 + OFFSET, 100.0 + OFFSET], [0, 100])
              ))
