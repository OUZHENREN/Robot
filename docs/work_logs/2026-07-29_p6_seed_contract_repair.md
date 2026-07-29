# P6 matched-noise repair and pilot invalidation — 2026-07-29

## Finding

The initial P6 formal attempt at
`/home/yff/nbv_p6_formal/20260729T075831Z` was stopped with exit code 130 and
is an **invalid pilot**, not P6 evidence.  The synthetic camera formed its
noise key as `random_seed + 7919 * view_index`, but `view_index` was not reset
between episodes.  Thus same-numbered seeds across strategies did not imply
the same synthetic sensor-noise realization.

No data were deleted.  The pilot export remains available for debugging and is
explicitly excluded from any report table, confidence interval, or conclusion.

## Repair

- Every `random_seed` parameter update now resets the synthetic camera's
  `view_index` and active-view state, including a parameter update whose value
  equals the launch default.
- The batch manifest and each episode `config.yaml` now record
  `sensor_noise_seed_contract:
  random_seed_plus_7919_times_view_index_reset_per_episode`.
- The P6 MATLAB analysis rejects an episode if that contract or the declared
  15 mm / 30 mm controlled initial condition is absent.

## Verification

`cs625_nbv` compiled on the VM.  The replacement two-strategy, same-seed,
controller-free virtual smoke batch at
`/home/yff/nbv_p6_seed_reset_smoke_v2/20260729T082658Z` passed:

- exit code 0; two `run_index.csv` entries, both seed 625;
- two log messages resetting the virtual sensor view epoch for seed 625;
- both episode configs contain the declared seed contract and controlled
  initial condition.

Only after this repair will the 3 × 3 × 20 P6 batch be restarted.
