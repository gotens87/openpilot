#!/usr/bin/env python3
import json
import subprocess

from pathlib import Path

from openpilot.starpilot.navigation.mapd_wrapper import CorruptTileMonitor, quarantine_offline_tile, terminate_child


def _loading_line(filename: str) -> str:
  return json.dumps({"msg": "Loading bounds file", "filename": filename})


def _error_line() -> str:
  return json.dumps({"msg": "could not unmarshal offline data", "error": "EOF"})


def test_corrupt_tile_monitor_triggers_after_repeated_failures():
  filename = "/data/media/0/osm/offline/36/-98/37.500000_-98.000000_37.750000_-97.750000"
  monitor = CorruptTileMonitor(threshold=3, window_s=3.0)

  assert monitor.observe(_loading_line(filename), now=0.0) is None
  assert monitor.observe(_error_line(), now=0.1) is None
  assert monitor.observe(_loading_line(filename), now=0.2) is None
  assert monitor.observe(_error_line(), now=0.3) is None
  assert monitor.observe(_loading_line(filename), now=0.4) is None
  assert monitor.observe(_error_line(), now=0.5) == filename


def test_quarantine_offline_tile_renames_file(tmp_path, monkeypatch):
  offline_root = tmp_path / "offline"
  tile = offline_root / "36/-98/37.500000_-98.000000_37.750000_-97.750000"
  tile.parent.mkdir(parents=True)
  tile.write_text("bad")

  monkeypatch.setitem(quarantine_offline_tile.__globals__, "OFFLINE_ROOT", offline_root)

  quarantined = quarantine_offline_tile(tile.as_posix())

  assert quarantined is not None
  assert not tile.exists()
  assert Path(quarantined).exists()
  assert Path(quarantined).name.startswith(f"{tile.name}.corrupt.")


def test_quarantine_offline_tile_renames_backing_archive(tmp_path, monkeypatch):
  offline_root = tmp_path / "offline"
  archive = offline_root / "34/-88.tar.gz"
  archive.parent.mkdir(parents=True)
  archive.write_text("bad archive")
  virtual_tile = offline_root / "34/-88/34.750000_-87.750000_35.000000_-87.500000"

  monkeypatch.setitem(quarantine_offline_tile.__globals__, "OFFLINE_ROOT", offline_root)

  quarantined = quarantine_offline_tile(virtual_tile.as_posix())

  assert quarantined is not None
  assert not archive.exists()
  assert Path(quarantined).exists()
  assert Path(quarantined).name.startswith(f"{archive.name}.corrupt.")


def test_terminate_child_tolerates_wedged_process():
  class WedgedProcess:
    pid = 123

    def __init__(self):
      self.terminated = False
      self.killed = False

    def poll(self):
      return None

    def terminate(self):
      self.terminated = True

    def kill(self):
      self.killed = True

    def wait(self, timeout):
      raise subprocess.TimeoutExpired("mapd", timeout)

  proc = WedgedProcess()
  terminate_child(proc)

  assert proc.terminated
  assert proc.killed
