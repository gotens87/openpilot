#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil

from collections import Counter
from pathlib import Path


SPEED_VALUES = frozenset((15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Add trusted later-frame sign tracks to a classifier dataset.")
  parser.add_argument("--base", type=Path, required=True)
  parser.add_argument("--track-samples", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--train-ratio", type=float, default=0.85)
  parser.add_argument("--min-growth", type=float, default=1.10)
  parser.add_argument("--min-exact-confidence", type=float, default=0.80)
  parser.add_argument("--min-detector-confidence", type=float, default=0.30)
  parser.add_argument("--max-track-rank", type=int, default=3)
  return parser.parse_args()


def split_for_key(key: str, train_ratio: float) -> str:
  fraction = int(hashlib.sha1(key.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
  return "train" if fraction < train_ratio else "val"


def link_or_copy(source: Path, destination: Path) -> None:
  destination.parent.mkdir(parents=True, exist_ok=True)
  if destination.exists():
    return
  try:
    os.link(source, destination)
  except OSError:
    shutil.copy2(source, destination)


def remove_appledouble_files(root: Path) -> int:
  removed = 0
  for path in root.rglob("._*"):
    if path.is_file():
      path.unlink()
      removed += 1
  return removed


def trusted_track_row(row: dict[str, str], args: argparse.Namespace) -> bool:
  try:
    expected = int(row.get("expected_speed_limit_mph", ""))
    predicted = int(row.get("predicted_speed_limit_mph", "") or 0)
    read_confidence = float(row.get("read_confidence", "") or 0.0)
    detector_confidence = float(row.get("detector_confidence", "") or 0.0)
    growth = float(row.get("area_ratio_to_anchor", "") or 0.0)
    rank = int(row.get("rank", "") or 999)
  except ValueError:
    return False
  exact = predicted == expected and read_confidence >= args.min_exact_confidence
  detector_snap = detector_confidence >= args.min_detector_confidence
  return expected in SPEED_VALUES and growth >= args.min_growth and rank <= args.max_track_rank and (exact or detector_snap)


def main() -> int:
  args = parse_args()
  base = args.base.expanduser().resolve()
  output = args.output.expanduser().resolve()
  counts: Counter[str] = Counter()
  validation_routes: set[str] = set()
  for split in ("train", "val"):
    for class_dir in (base / split).iterdir():
      if not class_dir.is_dir() or class_dir.name.startswith("._"):
        continue
      for source in class_dir.iterdir():
        if not source.is_file() or source.name.startswith("._") or source.suffix.lower() not in (".jpg", ".jpeg", ".png"):
          continue
        link_or_copy(source, output / split / class_dir.name / f"base_{source.name}")
        counts[f"base_{split}"] += 1

  with args.track_samples.expanduser().resolve().open(encoding="utf-8", newline="") as input_file:
    for row in csv.DictReader(input_file):
      if not trusted_track_row(row, args):
        counts["track_rejected"] += 1
        continue
      source = Path(row.get("crop_path", "")).expanduser().resolve()
      if not source.is_file():
        counts["track_rejected"] += 1
        continue
      speed = int(row["expected_speed_limit_mph"])
      split = split_for_key(row.get("route") or row.get("track_key", ""), args.train_ratio)
      if split == "val" and row.get("route"):
        validation_routes.add(row["route"])
      name = f"track_{row.get('track_key', '')}_{row.get('rank', '')}{source.suffix.lower()}"
      link_or_copy(source, output / split / str(speed) / name)
      counts[f"track_{split}"] += 1
      counts[f"speed_{speed}"] += 1

  counts["appledouble_removed"] = remove_appledouble_files(output)
  (output / "track_validation_routes.txt").write_text("\n".join(sorted(validation_routes)) + "\n", encoding="ascii")
  summary = {"base": str(base), "output": str(output), "counts": dict(sorted(counts.items()))}
  (output / "track_dataset_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="ascii")
  print(json.dumps(summary, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
