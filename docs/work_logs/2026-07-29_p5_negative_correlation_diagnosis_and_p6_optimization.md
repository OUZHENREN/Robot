# P5 negative-correlation diagnosis and P6 optimization — 2026-07-29

## Scope

All results in this log are from the virtual cuboid sensor.  No robot driver,
camera, gripper, IO, trajectory controller, real object, or CAD ADD/ADD-S
metric was used.

## Reproduced P5 diagnostic

The source dataset is the immutable P5 calibration export:

`Ubuntu_Share/report_exports/20260729_p5_calibration/{20260729T070351Z,20260729T070734Z,20260729T071211Z}`.

`matlab/analyze_nbv_p5_error_proxy_diagnostics.m` generated:

`Ubuntu_Share/matlab_output/20260729_p5_error_proxy_diagnostics/`.

| Slice | n transitions | r(predicted std reduction, observed covariance std reduction) | r(predicted std reduction, next-step truth-error reduction) | r(prior error, next-step truth-error reduction) | error worsened |
|---|---:|---:|---:|---:|---:|
| all | 43 | 0.696 | -0.253 | 0.678 | 55.8% |
| first view | 30 | 0.884 | -0.028 | 0.698 | 66.7% |
| later views | 13 | 0.724 | 0.204 | 0.618 | 30.8% |

## Interpretation

The original negative pooled correlation is not evidence that a higher
PoseGain score makes pose estimation worse.  The predictor targets a
translation covariance-standard-deviation reduction and tracks its aligned
covariance endpoint.  It was instead compared with a noisy, instantaneous
difference in ICP truth error, after mixing view numbers and occlusion levels.
P5 starts at millimetre-scale error, so numerical ICP variation often exceeds
the remaining error-reduction signal.  The error endpoint is therefore
exploratory and cannot support a strategy claim.

## Implemented P6 remedies

- Export `observed_covariance_translation_std_reduction_m` directly, rather
  than reconstructing it downstream.
- Export `virtual_initial_translation_bias_m` in every row.
- Add a default-off, virtual-only deterministic initial translation bias and
  an aggregate translation-standard-deviation prior.  The requested scalar
  standard deviation is distributed across three covariance axes so the CSV
  value remains the requested scalar (not `sqrt(3)` larger).
- Record `uncertainty_model` and `observability_model` from launch arguments
  into each episode `config.yaml`, rather than hard-coding P4 labels.
- Add a P6 preregistration and MATLAB diagnostic with aligned/unaligned
  endpoints kept separate.

## Verification

The VM build of `cs625_nbv` completed successfully.  A single, controller-free
pure virtual smoke test completed at:

`Ubuntu_Share/report_exports/20260729_p6_smoke_v2/20260729T075015Z`.

Its manifest records `controller_check_required: false`, 1 mm deterministic
depth noise, a 15 mm initial translation bias, and a 30 mm scalar prior
translation standard deviation.  The two exported steps recorded truth error
of 15.535 mm then 0.366 mm; this confirms the controlled measurement path,
not a strategy comparison.  Its `config.yaml` records
`p6_controlled_initial_error_virtual_only` and
`pose_aware_zbuffer_visibility`.

## Decision

P5 calibration remains a pass.  No P5 or P6 strategy-superiority, real-camera,
or hardware-performance conclusion is made.  P6 formal data collection must
use the preregistered matched-seed design before evaluating PoseGain.
