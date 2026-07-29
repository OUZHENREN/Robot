# P6 formal matched-seed virtual result — 2026-07-29

## Admissibility

The repaired formal export is at
`Ubuntu_Share/report_exports/20260729_p6_formal_r2/` and comprises three
successful batches (`20260729T083353Z`, `20260729T090802Z`,
`20260729T093634Z`).  It contains 180 episodes: none/light/heavy ×
fixed_order/random_reachable/pose_gain × 20 matched seeds (625–644).

All 180 episode configs contain the required virtual-only controlled initial
condition (15 mm bias, 30 mm scalar prior standard deviation) and the repaired
noise contract `random_seed_plus_7919_times_view_index_reset_per_episode`.
All 180 summaries have an empty failure reason.  The earlier interrupted
`nbv_p6_formal` directory remains an invalid pilot and is excluded.

## Frozen MATLAB analysis

`matlab/analyze_nbv_p6_formal.m` wrote its auditable output to
`Ubuntu_Share/matlab_output/20260729_p6_formal_r2/`.

The pre-registered sign convention is baseline final error minus PoseGain
final error; positive values favour PoseGain.

| Comparison | n | Median improvement (m) | Bootstrap 95% CI (m) |
|---|---:|---:|---:|
| pooled fixed_order − pose_gain | 60 | -0.000079 | [-0.000330, 0.000129] |
| pooled random_reachable − pose_gain | 60 | 0.000181 | [-0.000292, 0.000506] |
| heavy fixed_order − pose_gain | 20 | 0.000108 | [-0.000277, 0.000546] |
| heavy random_reachable − pose_gain | 20 | 0.000374 | [-0.000154, 0.000615] |

## Decision

The design is complete, but both pooled confidence intervals include zero.
P6 therefore **does not support PoseGain superiority** in this controlled
virtual benchmark.  It does not imply harm, real-camera performance,
hardware performance, or CAD ADD/ADD-S performance.

An initial parser bug treated a trailing empty CSV `failure_reason` as a
missing string.  It was fixed by normalising missing values to empty before
the design audit, then MATLAB was rerun.  This changed the design flag from
NO to YES and did not change any paired effects or confidence intervals.
