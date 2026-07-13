from collections import deque

import numpy as np
import pytest

import starpilot.system.speed_limit_vision as slv
from starpilot.system.speed_limit_vision import HistoryEntry, SpeedLimitVisionDaemon


def daemon_with_history(current_speed, entries):
  daemon = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  daemon.published_speed_limit_mph = current_speed
  daemon.history = deque(HistoryEntry(speed, confidence, float(index)) for index, (speed, confidence) in enumerate(entries))
  return daemon


def test_speed_change_requires_two_matching_reads():
  daemon = daemon_with_history(40, [(55, 0.95)])
  assert daemon._confirm_detection() is None

  daemon.history.append(HistoryEntry(55, 0.76, 1.0))
  assert daemon._confirm_detection() == pytest.approx((55, 0.95))


def test_speed_change_accepts_single_strong_consensus_read():
  daemon = daemon_with_history(70, [])
  daemon.history.append(HistoryEntry(60, 0.74, 1.0, strong_consensus=True))
  assert daemon._confirm_detection() == pytest.approx((60, 0.74))


def test_low_speed_change_requires_two_high_confidence_reads():
  daemon = daemon_with_history(40, [(25, 0.95)])
  assert daemon._confirm_detection() is None

  daemon.history.append(HistoryEntry(25, 0.96, 1.0))
  assert daemon._confirm_detection() == pytest.approx((25, 0.96))


def test_low_speed_change_accepts_single_strong_consensus_read():
  daemon = daemon_with_history(40, [])
  daemon.history.append(HistoryEntry(25, 0.95, 1.0, strong_consensus=True))
  assert daemon._confirm_detection() == pytest.approx((25, 0.95))


def test_low_speed_change_rejects_low_confidence_sequence():
  daemon = daemon_with_history(40, [(25, 0.82), (25, 0.88), (25, 0.89)])
  assert daemon._confirm_detection() is None


def detector_classifier_daemon(*, regulatory: bool, model_read, bbox=(700, 100, 780, 220), proposal_confidence=0.80):
  daemon = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  daemon._collect_detector_classifier_proposals = lambda _frame: [(proposal_confidence, 0, bbox)]
  daemon._is_regulatory_speed_sign = lambda _crop: regulatory
  daemon._classify_speed_limit_from_model = model_read if callable(model_read) else lambda _crop: model_read
  daemon._read_speed_limit_from_crop = lambda _crop: pytest.fail("detector/classifier runtime must not call OCR")
  return daemon


@pytest.fixture
def model_only_runtime(monkeypatch):
  monkeypatch.setattr(slv, "DETECTOR_CLASSIFIER_CROP_OCR_ENABLED", False)


def test_detector_classifier_runtime_reads_regulatory_sign_without_ocr(model_only_runtime):
  daemon = detector_classifier_daemon(regulatory=True, model_read=(55, 0.99))
  detection = daemon._detect_sign_from_detector_classifier(np.zeros((480, 960, 3), dtype=np.uint8))

  assert detection is not None
  assert detection.speed_limit_mph == 55


def test_detector_classifier_marks_two_strong_model_crops_as_consensus(model_only_runtime):
  reads = iter(((20, 0.96), (20, 0.97), None))
  daemon = detector_classifier_daemon(regulatory=True, model_read=lambda _crop: next(reads), proposal_confidence=0.80)
  detection = daemon._detect_sign_from_detector_classifier(np.zeros((480, 960, 3), dtype=np.uint8))

  assert detection is not None
  assert detection.speed_limit_mph == 20
  assert detection.strong_consensus


def test_detector_classifier_runtime_rejects_single_untrusted_non_regulatory_model_read_without_ocr(model_only_runtime):
  reads = iter(((55, 0.99), None, None, None))
  daemon = detector_classifier_daemon(regulatory=False, model_read=lambda _crop: next(reads))
  detection = daemon._detect_sign_from_detector_classifier(np.zeros((480, 960, 3), dtype=np.uint8))

  assert detection is None


def test_detector_classifier_runtime_accepts_repeated_model_only_consensus_without_ocr(model_only_runtime):
  daemon = detector_classifier_daemon(regulatory=False, model_read=(60, 0.99))
  detection = daemon._detect_sign_from_detector_classifier(np.zeros((480, 960, 3), dtype=np.uint8))

  assert detection is not None
  assert detection.speed_limit_mph == 60


def test_detector_classifier_runtime_rejects_tiny_model_only_consensus_without_ocr(model_only_runtime):
  daemon = detector_classifier_daemon(
    regulatory=True,
    model_read=(40, 0.99),
    bbox=(700, 100, 720, 125),
    proposal_confidence=0.14,
  )
  detection = daemon._detect_sign_from_detector_classifier(np.zeros((480, 960, 3), dtype=np.uint8))

  assert detection is None
