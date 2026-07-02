#!/usr/bin/env python3
from pathlib import Path

from openpilot.system.hardware import HARDWARE

BIG_UI_DEVICE_TYPES = ("tici", "tizi")
SMALL_UI_DEVICE_TYPE = "mici"


def _normalize_device_type(raw: str) -> str:
  device_type = raw.replace("\x00", "").strip().lower()
  if "comma " in device_type:
    device_type = device_type.rsplit("comma ", 1)[-1].strip()
  return device_type


def _device_tree_device_type() -> str | None:
  model_path = Path("/sys/firmware/devicetree/base/model")
  if not model_path.is_file():
    return None

  try:
    return _normalize_device_type(model_path.read_text(encoding="utf-8", errors="ignore"))
  except Exception:
    return None


def _reported_device_type() -> str | None:
  try:
    return _normalize_device_type(HARDWARE.get_device_type())
  except Exception:
    return None


def _ui_device_type() -> str:
  device_tree_type = _device_tree_device_type()
  if device_tree_type in BIG_UI_DEVICE_TYPES:
    return device_tree_type
  if device_tree_type:
    return SMALL_UI_DEVICE_TYPE

  reported_type = _reported_device_type()
  if reported_type in BIG_UI_DEVICE_TYPES:
    return reported_type
  return SMALL_UI_DEVICE_TYPE


def _patch_hardware_device_type(device_type: str) -> None:
  HARDWARE.get_device_type = lambda: device_type
  try:
    import openpilot.system.hardware.tici.hardware as tici_hardware
    tici_hardware.get_device_type = lambda: device_type
  except Exception:
    pass


def main():
  device_type = _ui_device_type()
  _patch_hardware_device_type(device_type)

  if device_type in BIG_UI_DEVICE_TYPES:
    import openpilot.system.ui.tici_reset as reset_impl
  else:
    import openpilot.system.ui.mici_reset as reset_impl

  reset_impl.main()


if __name__ == "__main__":
  main()
