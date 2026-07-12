#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json

from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2

import starpilot.system.speed_limit_vision as slv

if __package__ in (None, ""):
  import sys
  sys.path.insert(0, str(Path(__file__).resolve().parent))
  from import_manual_review_queue import merged_review_rows, parse_speed  # type: ignore
  from replay_route_runtime import RouteReplayDaemon, configure_models  # type: ignore
else:
  from .import_manual_review_queue import merged_review_rows, parse_speed
  from .replay_route_runtime import RouteReplayDaemon, configure_models


POSITIVE_STATUSES = frozenset(("accepted", "corrected"))


@dataclass(frozen=True)
class ReviewedCase:
  record_key: str
  route: str
  segment: int
  frame_time_s: float
  source_video_path: Path
  expected_speed_limit_mph: int
  negative: bool


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Evaluate reviewed sign events through realistic runtime cadence and publish logic.")
  parser.add_argument("--queue", type=Path, required=True, help="Reviewed manual_review_queue.csv.")
  parser.add_argument("--labels", type=Path, help="Defaults to manual_review_labels.csv beside the queue.")
  parser.add_argument("--models-dir", type=Path, default=Path("starpilot/assets/vision_models"), help="Candidate ONNX model directory.")
  parser.add_argument("--output-csv", type=Path, required=True, help="Per-event evaluation output.")
  parser.add_argument("--window-before", type=float, default=4.0, help="Seconds replayed before the reviewed frame.")
  parser.add_argument("--window-after", type=float, default=3.0, help="Seconds replayed after the reviewed frame.")
  parser.add_argument("--dedupe-seconds", type=float, default=3.0, help="Collapse nearby reviewed rows with the same expected value.")
  parser.add_argument("--measured-base-inference-seconds", type=float, default=0.44, help="Measured no-proposal comma inference cost.")
  parser.add_argument("--measured-classifier-forward-seconds", type=float, default=0.066, help="Measured comma cost per classifier forward.")
  parser.add_argument("--crop-ocr", action="store_true", help="Evaluate with crop OCR confirmation enabled.")
  parser.add_argument("--classifier-min-confidence", type=float, help="Override the value classifier confidence threshold.")
  parser.add_argument("--trusted-model-min-confidence", type=float, help="Override tiny-box trusted model confidence.")
  parser.add_argument("--strong-rescue-min-proposal-confidence", type=float, help="Override single-frame tiny-sign proposal confidence.")
  parser.add_argument("--strong-rescue-min-read-confidence", type=float, help="Override single-frame tiny-sign classifier confidence.")
  parser.add_argument("--low-speed-change-consistent-detections", type=int, help="Override reads required to change from 30+ mph to below 30 mph.")
  parser.add_argument(
    "--allow-low-speed-strong-consensus",
    action="store_true",
    help="Permit a strong multi-crop consensus to publish a low-speed change from one frame.",
  )
  parser.add_argument(
    "--enable-strong-model-consensus",
    action="store_true",
    help="Mark three agreeing high-confidence regulatory model crops as strong consensus.",
  )
  parser.add_argument("--initial-speed-limit", type=int, default=0, help="Seed each replay window with a currently published speed limit.")
  parser.add_argument("--positive-only", action="store_true", help="Replay only reviewed speed signs, omitting ignored-crop windows.")
  parser.add_argument("--max-cases", type=int, default=0, help="Optional evaluation cap after deduplication.")
  return parser.parse_args()


def load_cases(queue_path: Path, labels_path: Path, dedupe_seconds: float) -> list[ReviewedCase]:
  rows = merged_review_rows(queue_path, labels_path)
  cases: list[ReviewedCase] = []
  seen_buckets: set[tuple[str, int, int, int, bool]] = set()
  for row in rows:
    status = row.get("review_status", "")
    positive = status in POSITIVE_STATUSES
    negative = status == "ignore" and row.get("review_sign_type") == "not_speed_limit"
    if not positive and not negative:
      continue
    speed = parse_speed(row.get("review_speed_limit_mph", "")) if positive else 0
    if positive and not speed:
      continue
    try:
      segment = int(row.get("segment", ""))
      frame_time_s = float(row.get("frame_time_s", ""))
    except ValueError:
      continue
    source_video = Path(row.get("source_video_path", "")).expanduser()
    if not source_video.is_file():
      continue
    bucket = int(frame_time_s / max(dedupe_seconds, 0.1))
    dedupe_key = (row.get("route", ""), segment, bucket, speed, negative)
    if dedupe_key in seen_buckets:
      continue
    seen_buckets.add(dedupe_key)
    cases.append(ReviewedCase(
      record_key=row.get("record_key", ""),
      route=row.get("route", ""),
      segment=segment,
      frame_time_s=frame_time_s,
      source_video_path=source_video.resolve(),
      expected_speed_limit_mph=speed,
      negative=negative,
    ))
  return cases


def replay_video_cases(cases: list[ReviewedCase], args: argparse.Namespace) -> dict[str, tuple[list[int], list[int], int]]:
  daemons = {
    case.record_key: RouteReplayDaemon(
      runtime_context=None,
      measured_inference_seconds=0.0,
      measured_base_inference_seconds=args.measured_base_inference_seconds,
      measured_classifier_forward_seconds=args.measured_classifier_forward_seconds,
    )
    for case in cases
  }
  for daemon in daemons.values():
    daemon.published_speed_limit_mph = args.initial_speed_limit
    daemon.last_published_support_at = 0.0
  capture = cv2.VideoCapture(str(cases[0].source_video_path))
  fps = capture.get(cv2.CAP_PROP_FPS) or 20.0
  frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
  duration_s = frame_count / fps if frame_count > 0 else 60.0
  windows = {
    case.record_key: (
      max(case.frame_time_s - args.window_before, 0.0),
      min(case.frame_time_s + args.window_after, duration_s),
    )
    for case in cases
  }
  first_frame = max(int(min(window[0] for window in windows.values()) * fps), 0)
  end_frame = max(int(max(window[1] for window in windows.values()) * fps), first_frame)
  capture.set(cv2.CAP_PROP_POS_FRAMES, first_frame)
  frame_index = first_frame

  while frame_index <= end_frame:
    ok, frame_bgr = capture.read()
    if not ok:
      break
    frame_time_s = frame_index / fps
    for case in cases:
      start_s, end_s = windows[case.record_key]
      if start_s <= frame_time_s <= end_s:
        daemons[case.record_key].process_frame(frame_time_s - start_s, frame_bgr)
    frame_index += 1
  capture.release()

  results = {}
  for case in cases:
    daemon = daemons[case.record_key]
    candidates = [int(event["candidateSpeedLimitMph"]) for event in daemon.events if event["event"] == "candidate"]
    publishes = [int(event["speedLimitMph"]) for event in daemon.events if event["event"] == "publish"]
    results[case.record_key] = candidates, publishes, daemon.inference_frames
  return results


def main() -> int:
  args = parse_args()
  queue_path = args.queue.expanduser().resolve()
  labels_path = args.labels.expanduser().resolve() if args.labels else queue_path.with_name("manual_review_labels.csv")
  configure_models(args.models_dir)
  slv.DETECTOR_CLASSIFIER_CROP_OCR_ENABLED = args.crop_ocr
  if args.classifier_min_confidence is not None:
    slv.US_CLASSIFIER_MIN_CONFIDENCE = args.classifier_min_confidence
  if args.trusted_model_min_confidence is not None:
    slv.DETECTOR_CLASSIFIER_TRUSTED_MODEL_MIN_READ_CONFIDENCE = args.trusted_model_min_confidence
  if args.strong_rescue_min_proposal_confidence is not None:
    slv.DETECTOR_CLASSIFIER_STRONG_RESCUE_MIN_PROPOSAL_CONFIDENCE = args.strong_rescue_min_proposal_confidence
  if args.strong_rescue_min_read_confidence is not None:
    slv.DETECTOR_CLASSIFIER_STRONG_RESCUE_MIN_READ_CONFIDENCE = args.strong_rescue_min_read_confidence
  if args.low_speed_change_consistent_detections is not None:
    slv.LOW_SPEED_CHANGE_CONSISTENT_DETECTIONS = args.low_speed_change_consistent_detections
  if args.allow_low_speed_strong_consensus:
    slv.LOW_SPEED_CHANGE_ALLOW_STRONG_CONSENSUS = True
  if args.enable_strong_model_consensus:
    slv.DETECTOR_CLASSIFIER_STRONG_MODEL_CONSENSUS_ENABLED = True
  cases = load_cases(queue_path, labels_path, args.dedupe_seconds)
  if args.positive_only:
    cases = [case for case in cases if not case.negative]
  if args.max_cases > 0:
    cases = cases[:args.max_cases]

  output_rows: list[dict[str, object]] = []
  positive_by_speed: dict[int, Counter[str]] = defaultdict(Counter)
  negative_counts: Counter[str] = Counter()
  results: dict[str, tuple[list[int], list[int], int]] = {}
  cases_by_video: dict[Path, list[ReviewedCase]] = defaultdict(list)
  for case in cases:
    cases_by_video[case.source_video_path].append(case)
  for index, video_cases in enumerate(cases_by_video.values(), start=1):
    results.update(replay_video_cases(video_cases, args))
    if index % 10 == 0:
      print(f"Replayed {index}/{len(cases_by_video)} video segments", flush=True)

  for case in cases:
    candidates, publishes, inference_frames = results.get(case.record_key, ([], [], 0))
    candidate_hit = case.expected_speed_limit_mph in candidates if not case.negative else False
    publish_hit = case.expected_speed_limit_mph in publishes if not case.negative else False
    false_candidate = bool(candidates) if case.negative else False
    false_publish = bool(publishes) if case.negative else False
    if case.negative:
      negative_counts.update(total=1, candidate_fp=int(false_candidate), publish_fp=int(false_publish))
    else:
      positive_by_speed[case.expected_speed_limit_mph].update(
        total=1,
        candidate_hit=int(candidate_hit),
        publish_hit=int(publish_hit),
      )
    output_rows.append({
      "record_key": case.record_key,
      "route": case.route,
      "segment": case.segment,
      "frame_time_s": f"{case.frame_time_s:.3f}",
      "expected_speed_limit_mph": "" if case.negative else case.expected_speed_limit_mph,
      "negative": case.negative,
      "candidate_values": "|".join(str(value) for value in candidates),
      "publish_values": "|".join(str(value) for value in publishes),
      "candidate_hit": candidate_hit,
      "publish_hit": publish_hit,
      "false_candidate": false_candidate,
      "false_publish": false_publish,
      "inference_frames": inference_frames,
      "source_video_path": str(case.source_video_path),
    })
  output_path = args.output_csv.expanduser().resolve()
  output_path.parent.mkdir(parents=True, exist_ok=True)
  fieldnames = list(output_rows[0]) if output_rows else []
  with output_path.open("w", encoding="utf-8", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(output_rows)

  totals = Counter()
  for counts in positive_by_speed.values():
    totals.update(counts)
  negative_scope = " ".join((
    "Reviewed negative labels apply to the proposed crop.",
    "Candidate/publish counts replay the full video window and are an upper bound, not confirmed false positives,",
    "because another valid sign may be visible in that window.",
  ))
  summary = {
    "models_dir": str(args.models_dir.expanduser().resolve()),
    "crop_ocr": args.crop_ocr,
    "classifier_min_confidence": slv.US_CLASSIFIER_MIN_CONFIDENCE,
    "measured_base_inference_seconds": args.measured_base_inference_seconds,
    "measured_classifier_forward_seconds": args.measured_classifier_forward_seconds,
    "initial_speed_limit_mph": args.initial_speed_limit,
    "low_speed_change_consistent_detections": slv.LOW_SPEED_CHANGE_CONSISTENT_DETECTIONS,
    "low_speed_change_allow_strong_consensus": slv.LOW_SPEED_CHANGE_ALLOW_STRONG_CONSENSUS,
    "strong_model_consensus_enabled": slv.DETECTOR_CLASSIFIER_STRONG_MODEL_CONSENSUS_ENABLED,
    "positive": dict(totals),
    "positive_by_speed": {str(speed): dict(counts) for speed, counts in sorted(positive_by_speed.items())},
    "negative": dict(negative_counts),
    "negative_scope": negative_scope,
  }
  summary_path = output_path.with_suffix(".json")
  summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
