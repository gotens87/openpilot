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
| 6 | stopped-lead-promote | radard: promote near-stationary in-path lead sooner (relax fusion lateral gate) | experimental |
| 7 | range-rate-follow-TEST | long_mpc: range-rate cost term for follow distance (TESTING, unvalidated) | TESTING |

Log:
- 2026-06-13: rewrote 13 fine-grained commits -> these 5; squashed self-corrections
  (ff-scale add/undo -> net stock ff; radar status bit relabel folded into #1).
- 2026-06-21: added #6. Continental ARS suppresses stopped targets to ~46m; once the cluster
  appears, track_matches_vision's lateral gate kept leadOne on the noisy vision lead to ~22m
  (~1.9s late). Relax y_floor 1.0->2.5 for confident (prob>0.6) near-stationary in-path tracks.
  DRIVE-TEST PENDING: watch adjacent-lane stopped-car phantom braking at red lights.
- 2026-06-21: added #7 (TESTING). Restored the orphaned range-rate term (was 3661880) from git
  objects incl. its matching acados binaries; PARAM_DIM 6->8, V_REL_COST=4.0; acados init clean.
  Targets too-far FOLLOW on stable moving leads, NOT the stopped-car path (#6/#42). DRIVE-TEST PENDING.
