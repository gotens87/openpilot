import importlib
import sys
import types
from unittest.mock import patch

import pytest


@pytest.fixture
def reset_module(monkeypatch):
  hardware = types.ModuleType("openpilot.system.hardware")
  hardware.HARDWARE = types.SimpleNamespace(get_device_type=lambda: "tici")
  monkeypatch.setitem(sys.modules, "openpilot.system.hardware", hardware)
  sys.modules.pop("openpilot.system.ui.reset", None)
  module = importlib.import_module("openpilot.system.ui.reset")
  yield module
  sys.modules.pop("openpilot.system.ui.reset", None)


def test_device_tree_tici_uses_big_ui(reset_module):
  with patch.object(reset_module, "_device_tree_device_type", return_value="tici"), \
       patch.object(reset_module, "_reported_device_type", return_value="mici"):
    assert reset_module._ui_device_type() == "tici"


def test_device_tree_tizi_uses_big_ui(reset_module):
  with patch.object(reset_module, "_device_tree_device_type", return_value="tizi"), \
       patch.object(reset_module, "_reported_device_type", return_value="mici"):
    assert reset_module._ui_device_type() == "tizi"


@pytest.mark.parametrize("device_tree_type", ["mici", "comma 4", "comma four"])
def test_non_tici_device_tree_uses_small_ui(reset_module, device_tree_type):
  with patch.object(reset_module, "_device_tree_device_type", return_value=device_tree_type), \
       patch.object(reset_module, "_reported_device_type", return_value="tici"):
    assert reset_module._ui_device_type() == "mici"


def test_reported_tici_used_when_no_device_tree(reset_module):
  with patch.object(reset_module, "_device_tree_device_type", return_value=None), \
       patch.object(reset_module, "_reported_device_type", return_value="tici"):
    assert reset_module._ui_device_type() == "tici"
