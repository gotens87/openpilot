#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil

from collections import Counter
from pathlib import Path


VALID_SPEEDS = frozenset((15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build an isolated classifier dataset from a base corpus and reviewed crops.")
  parser.add_argument("--base", type=Path, required=True, help="Existing Ultralytics classification dataset root.")
  parser.add_argument("--output", type=Path, required=True, help="New isolated dataset root.")
  parser.add_argument("--positive-manifest", type=Path, action="append", default=[], help="Reviewed positive crop manifest. Repeat as needed.")
  parser.add_argument("--reject-manifest", type=Path, action="append", default=[], help="Reviewed classifier reject manifest. Repeat as needed.")
  parser.add_argument(
    "--advisory-as-reject",
    action="store_true",
    help="Stage reviewed advisory-speed crops in the reject class instead of omitting them.",
  )
  parser.add_argument(
    "--include-advisory-positives",
    action="store_true",
    help="Train reviewed advisory crops as their numeric speed classes for recall-first models.",
  )
  parser.add_argument(
    "--advisory-reject-fraction",
    type=float,
    default=1.0,
    help="Deterministic fraction of training advisories staged as reject; validation advisories are always retained.",
  )
  return parser.parse_args()


def read_rows(paths: list[Path]):
  for path in paths:
    resolved = path.expanduser().resolve()
    with resolved.open("r", encoding="utf-8", newline="") as handle:
      yield from csv.DictReader(handle)


def remove_appledouble_files(root: Path) -> int:
  removed = 0
  for path in root.rglob("._*"):
    if path.is_file():
      path.unlink()
      removed += 1
  return removed


def parse_speed(text: str) -> int:
  try:
    value = int(float((text or "").strip()))
  except ValueError:
    return 0
  return value if value in VALID_SPEEDS else 0


def is_advisory(row: dict[str, str]) -> bool:
  return row.get("review_sign_type", "").strip().lower() == "advisory"


def keep_advisory_reject(row: dict[str, str], fraction: float) -> bool:
  if row.get("split") == "val" or fraction >= 1.0:
    return True
  if fraction <= 0.0:
    return False
  digest = hashlib.sha256(row.get("record_key", "").encode("utf-8")).digest()
  return int.from_bytes(digest[:8], "big") / 2**64 < fraction


def stage_crop(source: Path, destination_dir: Path, record_key: str) -> bool:
  if not source.is_file():
    return False
  digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
  suffix = source.suffix.lower() if source.suffix.lower() in (".jpg", ".jpeg", ".png") else ".jpg"
  safe_key = "".join(char if char.isalnum() or char in "._-" else "_" for char in record_key)[:100]
  destination_dir.mkdir(parents=True, exist_ok=True)
  destination = destination_dir / f"review_{safe_key}_{digest}{suffix}"
  if not destination.exists():
    shutil.copyfile(source, destination)
  return True


def main() -> int:
  args = parse_args()
  if args.advisory_as_reject and args.include_advisory_positives:
    raise ValueError("--advisory-as-reject and --include-advisory-positives are mutually exclusive")
  if not 0.0 <= args.advisory_reject_fraction <= 1.0:
    raise ValueError("--advisory-reject-fraction must be between 0 and 1")
  base = args.base.expanduser().resolve()
  output = args.output.expanduser().resolve()
  if not base.is_dir():
    raise FileNotFoundError(base)
  if output.exists():
    raise FileExistsError(f"Output dataset already exists: {output}")
  shutil.copytree(base, output, copy_function=shutil.copyfile)
  appledouble_removed = remove_appledouble_files(output)

  positive_counts: Counter[str] = Counter()
  reject_counts: Counter[str] = Counter()
  skipped = 0
  for row in read_rows(args.positive_manifest):
    if is_advisory(row):
      if args.include_advisory_positives:
        pass
      elif args.advisory_as_reject and keep_advisory_reject(row, args.advisory_reject_fraction):
        split = row.get("split", "")
        source = Path(row.get("crop_path", "")).expanduser()
        if split in ("train", "val") and stage_crop(source, output / split / "reject", row.get("record_key", "advisory")):
          reject_counts[f"advisory_{split}"] += 1
        else:
          skipped += 1
        continue
      else:
        continue
    split = row.get("split", "")
    speed = parse_speed(row.get("speed_limit_mph", ""))
    source = Path(row.get("crop_path", "")).expanduser()
    if split not in ("train", "val") or not speed or not stage_crop(source, output / split / str(speed), row.get("record_key", "positive")):
      skipped += 1
      continue
    positive_counts[f"{split}/{speed}"] += 1

  for row in read_rows(args.reject_manifest):
    split = row.get("split", "")
    source = Path(row.get("crop_path", "")).expanduser()
    if split not in ("train", "val") or not stage_crop(source, output / split / "reject", row.get("record_key", "reject")):
      skipped += 1
      continue
    reject_counts[split] += 1

  appledouble_removed += remove_appledouble_files(output)
  for split in ("train", "val"):
    cache_path = output / f"{split}.cache"
    if cache_path.is_file():
      cache_path.unlink()

  summary = {
    "base": str(base),
    "output": str(output),
    "positive_counts": dict(sorted(positive_counts.items())),
    "reject_counts": dict(sorted(reject_counts.items())),
    "skipped": skipped,
    "appledouble_removed": appledouble_removed,
  }
  summary_path = output / "review_dataset_summary.json"
  summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
