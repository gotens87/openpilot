# StarPilot Patch Stack - Camry

Upstream: firestar5683/StarPilot. Our patches cherry-pick onto upstream's tip each
update; SHAs change, so THIS manifest + the subject lines are the stable identity.
To drop a tune: remove its row + the commit. Keep each concern to ONE commit.

| # | Name | Subject | Status |
|---|------|---------|--------|
| 1 | camry-conti-radar | toyota: Camry Continental radar port (NO_DSU TSS-P) | required |
| 2 | camry-enablement  | toyota: Camry enablement - hybrid detect + NO_STOP_TIMER SNG | required |
| 3 | camry-long-tune   | toyota: Camry longitudinal tune (Prius THS-II PID) | experimental |
| 4 | camry-stops       | toyota: Camry smoother stops (eCVT stop ramp) | ride-quality |
| 5 | konik-gate        | infra: konik upload-gate (athenad/uploads to Konik not comma) | infra |

Parked (archived out of stack; re-add = git apply the patch, commit atomic, one concern per commit):
- mapd-EIO survival -> patches/mapd-eio-2026-07-06-a840584.patch (failing-nvme workaround)
- onroad reboot + poweroff + UI self-heal -> patches/reboot-poweroff-selfheal-2026-07-06-a840584.patch
  (the self-heal check BOOT-LOOPS the car on a base it can't rebuild; do not re-add until the
   embed-hash-into-rebuild path is fixed)

Log:
- 2026-06-13: rewrote 13 fine-grained commits -> these 5; squashed self-corrections.
- 2026-07-06: split bundled star-infra (#5); konik-gate stays in-stack, mapd-EIO +
  reboot/poweroff/self-heal parked to patches/. Boot loop traced to the self-heal check
  forcing an on-device rebuild of a WIP BigUI base that never succeeds.
