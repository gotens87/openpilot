# StarPilot patch queue

This branch (`ops`) is the control plane. It shares no history with openpilot.

`.github/workflows/rebuild-starpilot.yml` runs every 10 minutes and rebuilds branch
`StarPilot` as **upstream `firestar5683/StarPilot` `Dom` tip + `ops_patches/patch_*.py`**,
force-pushing the result. The device tracks `StarPilot` and auto-updates, so local
changes survive every upstream move instead of being wiped by the updater's hard reset.

`ops` is the repo default branch on purpose: `schedule:` only fires from the default
branch, and `StarPilot` is force-reset to upstream content on every run, which would
wipe a workflow stored there.

## Patches

| file | effect |
| --- | --- |
| `starpilot/controls/starpilot_card.py` | auto-offroad entry when parked and stopped |
| `system/hardware/hardwared.py` | auto-offroad exit on leaving park (raw-CAN gear read) |
| `starpilot/controls/lib/conditional_experimental_mode.py` | highway light-boost multiplier 1.2 -> 1.0 |

`tools/apply-patchers.sh` owns the patcher list and the commit subjects, and is what
CI runs. The patchers are anchor-text `str.replace`, **one line per anchor**, inserting
after the anchor. Multi-line anchors die on any insertion inside their span; that broke
this stack twice in two days (Dom `acf32365b` grew an `__init__`, `decf85985` inserted
into a loop body).

There are no stored `.patch` files. They were a cache of one application of the
patchers and went stale on pure line-number drift.

## Editing a patch

Edit `ops_patches/patch_*.py`, push to `ops`, then dispatch with `force=true` — the
cheap moved-check only watches upstream, so a patcher edit alone will not rebuild.

```
gh workflow run rebuild-starpilot.yml --repo gotens87/openpilot --ref ops -f force=true
```

To preview what the patchers produce, run them against a throwaway checkout:

```
git clone --depth=1 -b Dom https://github.com/firestar5683/StarPilot.git /tmp/x
tools/apply-patchers.sh /tmp/x ./ops_patches && git -C /tmp/x log -p -3
```

## Failure mode

If a patcher's anchor is missing or ambiguous it exits non-zero, and the run **aborts
without pushing**. `StarPilot` stays at the last good build, so the device keeps
running that build with its patches intact. Fix the anchor, then dispatch with
`force=true`. Do not add auto-skip or auto-resolve — a silently dropped patch is the
failure this repo exists to prevent.

A patcher that applies but emits invalid Python is caught by the compile check at the
end of `apply-patchers.sh`, before anything is pushed.

## Requirements

- Repo secret `PATCHQUEUE_TOKEN`: PAT with `repo` + `workflow` scope. The built-in
  `GITHUB_TOKEN` cannot be used: the mirrored upstream tree contains files under
  `.github/workflows/`, and pushes touching those are refused for Actions tokens.
- Scheduled workflows are disabled by default in forks; enable once in the Actions tab.
- GitHub disables schedules after 60 days without repo activity. The rebuild's own
  pushes reset that clock.
