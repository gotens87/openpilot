#!/usr/bin/env bash
# apply-patchers.sh — apply ops_patches/patch_*.py to a checkout, one commit each.
#
# Single source of truth for the patcher list AND the commit subjects. Both the
# rebuild Action and tools/make-patches.sh call this, so the two can never drift.
#
#   apply-patchers.sh <checkout-dir> [patchers-dir]   apply + commit
#   apply-patchers.sh --count                          print patcher count, exit
#
# Caller owns git identity and dates (the Action pins GIT_COMMITTER_DATE to Dom's
# so an unchanged rebuild is bit-identical; make-patches.sh pins both to a fixed
# epoch so regenerated patches are byte-stable).
#
# Why this runs the patchers instead of `git am` on stored patches/*.patch:
# patch_*.py IS the patch; patches/*.patch was only a cache of one application of
# it. git am against that cache added line-number + blob-hash sensitivity and no
# semantic checking the patchers don't already do -- on 2026-08-11 an upstream
# insert that shifted line numbers in starpilot_card.py failed 6 consecutive
# scheduled runs with every anchor still present verbatim. Anchor-text matching
# is immune to that class.
#
# Fail-closed is preserved: a patcher whose anchor is missing/ambiguous exits
# non-zero, this script exits non-zero, the Action halts WITHOUT pushing, and the
# car keeps running the last-good build with its patches. Nothing is ever skipped
# or auto-resolved.
set -euo pipefail

PATCHERS=(
  patch_starpilot_card.py
  patch_hardwared.py
  patch_cem_lightboost.py
  patch_radar_valid.py
  patch_radar_yaw.py
)
SUBJECTS=(
  "starpilot_card: auto-offroad on park (entry)"
  "hardwared: auto-offroad exit on leaving park"
  "CEM: neutralize highway light-boost multiplier"
  "toyota: honour side-cluster VALID bit on TSS-P Continental radar"
  "toyota: tunable radar boresight yaw correction (default 0.0 = stock)"
)

if [ "${1:-}" = "--count" ]; then
  echo "${#PATCHERS[@]}"
  exit 0
fi

CHECKOUT=${1:?usage: apply-patchers.sh <checkout-dir> [patchers-dir] | --count}
PATCH_DIR=${2:-$(cd "$(dirname "$0")/.." && pwd)/ops_patches}

[ -d "$CHECKOUT" ] || { echo "apply-patchers: no checkout at $CHECKOUT" >&2; exit 1; }
[ "${#PATCHERS[@]}" -eq "${#SUBJECTS[@]}" ] || { echo "apply-patchers: PATCHERS/SUBJECTS length mismatch" >&2; exit 1; }

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

# The patchers hardcode device paths under /data/openpilot; retarget them at the
# checkout. The /data/ops_patches marker path is deliberately untouched -- that is
# a runtime path on the device, not a source path.
for f in "${PATCHERS[@]}"; do
  [ -f "$PATCH_DIR/$f" ] || { echo "apply-patchers: missing $PATCH_DIR/$f" >&2; exit 1; }
  sed "s|/data/openpilot|$CHECKOUT|g" "$PATCH_DIR/$f" > "$STAGE/$f"
done

cd "$CHECKOUT"
START=$(git rev-parse HEAD)
for i in "${!PATCHERS[@]}"; do
  echo "=== ${PATCHERS[$i]} ==="
  python3 "$STAGE/${PATCHERS[$i]}"
  git add -A
  # --allow-empty keeps commit count == patcher count, which the Action's
  # cheap "did upstream move" check relies on (it reads tip-minus-N as the last
  # built base). An empty commit means upstream adopted our change: surfaced as a
  # warning, not a failure, per the drop-superseded-patches convention.
  if git diff --cached --quiet; then
    echo "::warning::${PATCHERS[$i]} made no change - upstream may have adopted it; candidate for removal"
  fi
  git commit -q --allow-empty -m "${SUBJECTS[$i]}"
done

# A patcher can match its anchor and still emit broken Python (a mis-escaped quote
# in a replacement block does exactly that). Nothing else in the pipeline would catch
# it before the car booted on it. Builtin compile() = syntax check with no bytecode
# written, so no __pycache__ to clean up.
for f in $(git diff --name-only "$START"..HEAD | grep '\.py$' || true); do
  python3 -c "import sys; compile(open(sys.argv[1]).read(), sys.argv[1], 'exec')" "$f" \
    || { echo "::error::patched $f does not compile"; exit 1; }
done

echo "applied ${#PATCHERS[@]} patchers"
