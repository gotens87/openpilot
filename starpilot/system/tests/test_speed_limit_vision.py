from collections import deque

import pytest

from starpilot.system.speed_limit_vision import HistoryEntry, SpeedLimitVisionDaemon


def daemon_with_history(current_speed, entries):
  daemon = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  daemon.published_speed_limit_mph = current_speed
  daemon.history = deque(HistoryEntry(speed, confidence, float(index)) for index, (speed, confidence) in enumerate(entries))
  return daemon


def test_speed_change_requires_two_matching_reads():
  daemon = daemon_with_history(40, [(55, 0.70)])
  assert daemon._confirm_detection() is None

  daemon.history.append(HistoryEntry(55, 0.76, 1.0))
  assert daemon._confirm_detection() == pytest.approx((55, 0.76))


def test_speed_change_accepts_single_strong_consensus_read():
  daemon = daemon_with_history(70, [])
  daemon.history.append(HistoryEntry(60, 0.74, 1.0, strong_consensus=True))
  assert daemon._confirm_detection() == pytest.approx((60, 0.74))


def test_low_speed_change_requires_three_high_confidence_reads():
  daemon = daemon_with_history(40, [(25, 0.95), (25, 0.96)])
  assert daemon._confirm_detection() is None

  daemon.history.append(HistoryEntry(25, 0.94, 2.0))
  assert daemon._confirm_detection() == pytest.approx((25, 0.96))


def test_low_speed_change_ignores_single_strong_consensus_read():
  daemon = daemon_with_history(40, [])
  daemon.history.append(HistoryEntry(25, 0.95, 1.0, strong_consensus=True))
  assert daemon._confirm_detection() is None


def test_low_speed_change_rejects_low_confidence_sequence():
  daemon = daemon_with_history(40, [(25, 0.82), (25, 0.88), (25, 0.89)])
  assert daemon._confirm_detection() is None
