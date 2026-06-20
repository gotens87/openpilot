# StarPilot Patch Stack - Camry

Upstream: firestar5683/StarPilot. Our patches cherry-pick onto upstream's tip each
update; SHAs change, so THIS manifest + the subject lines are the stable identity.
To drop a tune: remove its row + the commit.

| # | Name | Subject | Status |
|---|------|---------|--------|
| 1 | camry-conti-radar | toyota: Camry Continental radar port (NO_DSU TSS-P) | required |
| 2 | camry-enablement  | toyota: Camry enablement - hybrid detect + NO_STOP_TIMER SNG | required |
| 3 | camry-long-tune   | toyota: Camry longitudinal tune (Prius THS-II PID) | experimental |
| 4 | camry-stops       | toyota: Camry smoother stops (eCVT stop ramp) | ride-quality |
| 5 | star-infra        | infra: StarPilot local (konik upload-gate + onroad reboot + onroad poweroff visibility + mapd EIO survival) | infra |

Log:
- 2026-06-13: rewrote 13 fine-grained commits -> these 5; squashed self-corrections
  (ff-scale add/undo -> net stock ff; radar status bit relabel folded into #1).
