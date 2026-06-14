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
| 5 | star-infra        | infra: StarPilot local (konik upload-gate + onroad reboot + mapd EIO survival) | infra |
| 6 | assert-normalcy   | assert normalcy: revert StarPilot's bespoke longitudinal layer to stock | required |

Log:
- 2026-06-13: rewrote 13 fine-grained commits -> these 5; squashed self-corrections
  (ff-scale add/undo -> net stock ff; radar status bit relabel folded into #1).
- 2026-06-14: + #6 "assert normalcy" - force stock get_max_accel / A_CRUISE_MIN +
  neutralize accel+speed jerk multipliers + far-lead throttle-kill = SunnyPilot/stock-
  equivalent longitudinal (kills the with-lead follow oscillation, a FrogPilot-lineage
  accel-layer regression). danger_jerk (braking) + t_follow left as-is. Drive-test pending.
