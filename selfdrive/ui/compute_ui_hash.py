#!/usr/bin/env python3
"""Compute a deterministic SHA-256 of the inputs used to build selfdrive/ui/ui.

The SConscript embeds this hash in the binary at build time; the boot check
(launch_chffrplus.sh) recomputes it from the working tree to verify the
committed prebuilt binary is fresh. Uses `git ls-files` (tracked + untracked
non-ignored) rather than a raw directory walk so build artifacts generated
mid-compile (moc output, .o/.d files) can never pollute the hash, even when
this runs concurrently with a parallel scons build.
"""
import hashlib
import subprocess
from pathlib import Path

UI_DIR = Path(__file__).resolve().parent

_EXCLUDE_NAMES = {"ui", "ui_build_hash.cc"}


def compute_ui_hash() -> str:
    """Return SHA-256 hex of all UI source inputs, sorted by relative path."""
    files = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=str(UI_DIR), text=True,
    ).splitlines()

    h = hashlib.sha256()
    for rel in sorted(files):
        if rel in _EXCLUDE_NAMES:
            continue
        path = UI_DIR / rel
        if not path.is_file():
            continue
        h.update(rel.encode("utf-8"))
        h.update(b"\0")
        h.update(path.read_bytes())
        h.update(b"\0")
    return h.hexdigest()


if __name__ == "__main__":
    print(compute_ui_hash())
