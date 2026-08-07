#!/usr/bin/env bash
# Regenerate patches/*.patch from the on-device Python patcher scripts.
# Run manually after a patcher changes; not used by CI.
set -euo pipefail

PATCH_DIR="${1:-./ops_patches}"
UPSTREAM=https://github.com/firestar5683/StarPilot.git
BRANCH=Dom

PATCHERS=(
  patch_starpilot_card.py
  patch_hardwared.py
  patch_cem_lightboost.py
)
SUBJECTS=(
  "starpilot_card: auto-offroad on park (entry)"
  "hardwared: auto-offroad exit on leaving park"
  "CEM: neutralize highway light-boost multiplier"
)

OUT=$(cd "$(dirname "$0")/.." && pwd)/patches
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# The patchers hardcode device paths under /data/openpilot; retarget them at the
# throwaway checkout. The /data/ops_patches marker path is deliberately untouched:
# it is a runtime path on the device, not a source path.
mkdir -p "$TMP/patchers"
for f in "${PATCHERS[@]}"; do
  sed "s|/data/openpilot|$TMP/checkout|g" "$PATCH_DIR/$f" > "$TMP/patchers/$f"
done

git clone --depth=1 --no-tags --single-branch --branch "$BRANCH" "$UPSTREAM" "$TMP/checkout"
cd "$TMP/checkout"

# Fixed identity + dates so regenerating without a content change is byte-identical.
export GIT_AUTHOR_NAME=gotens87
export GIT_AUTHOR_EMAIL=gotens87@users.noreply.github.com
export GIT_COMMITTER_NAME=$GIT_AUTHOR_NAME
export GIT_COMMITTER_EMAIL=$GIT_AUTHOR_EMAIL
export GIT_AUTHOR_DATE="2026-01-01T00:00:00Z"
export GIT_COMMITTER_DATE="$GIT_AUTHOR_DATE"

for i in "${!PATCHERS[@]}"; do
  echo "=== ${PATCHERS[$i]} ==="
  # A patcher that cannot find its anchor exits non-zero. Silently skipping one is
  # the exact failure this whole rig exists to prevent, so let set -e kill the run.
  python3 "$TMP/patchers/${PATCHERS[$i]}"
  git add -A
  git commit -q -m "${SUBJECTS[$i]}"
done

mkdir -p "$OUT"
rm -f "$OUT"/*.patch
# --zero-commit and --no-signature strip the base SHA and the git version, the two
# fields that would otherwise change the patch bytes without the content changing.
git format-patch -q --zero-commit --no-signature -"${#PATCHERS[@]}" -o "$OUT"

echo "wrote:"
ls -1 "$OUT"
