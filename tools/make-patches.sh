#!/usr/bin/env bash
# make-patches.sh — LOCAL PREVIEW ONLY. Regenerate patches/*.patch from the
# patcher scripts, to eyeball what the patchers actually produce against Dom.
#
# NOT the build path and NOT committed: since 2026-08-11 the rebuild Action runs
# tools/apply-patchers.sh directly against the upstream checkout. Stored .patch
# files were only a cache of one application of the patchers, and their hunk
# context went stale on any upstream line-number shift (6 consecutive failed runs
# 2026-08-11 with every anchor still present). patches/ is gitignored.
#
# Patcher list and commit subjects live in tools/apply-patchers.sh, not here.
set -euo pipefail

PATCH_DIR="${1:-./ops_patches}"
UPSTREAM=https://github.com/firestar5683/StarPilot.git
BRANCH=Dom

ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT="$ROOT/patches"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

git clone --depth=1 --no-tags --single-branch --branch "$BRANCH" "$UPSTREAM" "$TMP/checkout"
cd "$TMP/checkout"
git config user.name gotens87
git config user.email gotens87@users.noreply.github.com

# Fixed identity + dates so regenerating without a content change is byte-identical.
export GIT_AUTHOR_NAME=gotens87
export GIT_AUTHOR_EMAIL=gotens87@users.noreply.github.com
export GIT_COMMITTER_NAME=$GIT_AUTHOR_NAME
export GIT_COMMITTER_EMAIL=$GIT_AUTHOR_EMAIL
export GIT_AUTHOR_DATE="2026-01-01T00:00:00Z"
export GIT_COMMITTER_DATE="$GIT_AUTHOR_DATE"

COUNT=$("$ROOT/tools/apply-patchers.sh" --count)
"$ROOT/tools/apply-patchers.sh" "$TMP/checkout" "$(cd "$PATCH_DIR" && pwd)"

mkdir -p "$OUT"
rm -f "$OUT"/*.patch
# --zero-commit and --no-signature strip the base SHA and the git version, the two
# fields that would otherwise change the patch bytes without the content changing.
git format-patch -q --zero-commit --no-signature -"$COUNT" -o "$OUT"

echo "wrote (preview, gitignored):"
ls -1 "$OUT"
