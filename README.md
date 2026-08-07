# StarPilot patch queue

This branch (`ops`) is the control plane. It shares no history with openpilot.

`.github/workflows/rebuild-starpilot.yml` runs every 3 hours and rebuilds branch
`StarPilot` as **upstream `firestar5683/StarPilot` `Dom` tip + `patches/*.patch`**,
force-pushing the result. The device tracks `StarPilot` and auto-updates, so local
changes survive every upstream move instead of being wiped by the updater's hard reset.

`ops` is the repo default branch on purpose: `schedule:` only fires from the default
branch, and `StarPilot` is force-reset to upstream content on every run, which would
wipe a workflow stored there.

## Patches

| # | file | effect |
| --- | --- | --- |
| 0001 | `starpilot/controls/starpilot_card.py` | auto-offroad entry when parked and stopped |
| 0002 | `system/hardware/hardwared.py` | auto-offroad exit on leaving park (raw-CAN gear read) |
| 0003 | `starpilot/controls/lib/conditional_experimental_mode.py` | highway light-boost multiplier 1.2 -> 1.0 |

Regenerate from the device-side patcher scripts:

```
tools/make-patches.sh /path/to/ops_patches
```

Output is deterministic; identical input produces byte-identical patch files.

## Failure mode

If `git am` fails because upstream restructured a patched file, the run **aborts
without pushing**. `StarPilot` stays at the last good build, so the device keeps
running that build with its patches intact. Fix the patch, then re-run the workflow.
Do not add auto-skip or auto-resolve — a silently dropped patch is the failure this
repo exists to prevent.

## Requirements

- Repo secret `PATCHQUEUE_TOKEN`: PAT with `repo` + `workflow` scope. The built-in
  `GITHUB_TOKEN` cannot be used: the mirrored upstream tree contains files under
  `.github/workflows/`, and pushes touching those are refused for Actions tokens.
- Scheduled workflows are disabled by default in forks; enable once in the Actions tab.
- GitHub disables schedules after 60 days without repo activity. The rebuild's own
  pushes reset that clock.
