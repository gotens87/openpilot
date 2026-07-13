from __future__ import annotations

import importlib.util

from argparse import Namespace
from pathlib import Path


def load_local_module(name: str):
  path = Path(__file__).resolve().with_name(f"{name}.py")
  spec = importlib.util.spec_from_file_location(f"test_local_{name}", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


track_dataset = load_local_module("build_track_proposal_detector_dataset")
split_for_key = track_dataset.split_for_key
trusted_track_row = track_dataset.trusted_track_row


def args() -> Namespace:
  return Namespace(min_exact_confidence=0.8, min_detector_confidence=0.3, min_growth=1.0, max_track_rank=4)


def test_split_for_key_keeps_route_samples_together():
  assert split_for_key("route-a", 0.85) == split_for_key("route-a", 0.85)


def test_trusted_track_requires_current_review_speed_to_match():
  track = {
    "expected_speed_limit_mph": "45",
    "predicted_speed_limit_mph": "45",
    "read_confidence": "0.99",
    "detector_confidence": "0.0",
    "area_ratio_to_anchor": "1.2",
    "rank": "1",
  }
  assert trusted_track_row(track, {"review_speed_limit_mph": "45"}, args())
  assert not trusted_track_row(track, {"review_speed_limit_mph": "55"}, args())


def test_trusted_track_accepts_detector_snap_without_classifier_read():
  track = {
    "expected_speed_limit_mph": "35",
    "predicted_speed_limit_mph": "",
    "read_confidence": "",
    "detector_confidence": "0.7",
    "area_ratio_to_anchor": "1.0",
    "rank": "4",
  }
  assert trusted_track_row(track, {"review_speed_limit_mph": "35"}, args())


def test_trusted_track_rejects_low_growth_and_rank():
  track = {
    "expected_speed_limit_mph": "35",
    "predicted_speed_limit_mph": "35",
    "read_confidence": "0.99",
    "detector_confidence": "0.7",
    "area_ratio_to_anchor": "0.9",
    "rank": "5",
  }
  assert not trusted_track_row(track, {"review_speed_limit_mph": "35"}, args())


def test_trusted_track_rejects_tracks_from_redrawn_anchor():
  track = {
    "expected_speed_limit_mph": "35",
    "predicted_speed_limit_mph": "35",
    "read_confidence": "0.99",
    "detector_confidence": "0.7",
    "area_ratio_to_anchor": "1.2",
    "rank": "1",
  }
  review = {
    "review_speed_limit_mph": "35",
    "bbox": "10,10,40,50",
    "review_bbox": "12,12,35,45",
  }
  assert not trusted_track_row(track, review, args())
