# P6 — Controlled virtual error-reduction experiment (preregistered design)

## Why P6 exists

P5 validated the pose-aware virtual visibility predictor against the synthetic
sensor and stopped the former covariance-collapse artefact.  It did **not**
validate a policy effect on absolute pose error: P5 began from millimetre-scale
ICP error, mixed first and later views, and compared a predicted covariance
standard-deviation reduction with an instantaneous error difference.

P6 tests a different, explicit question: under a controlled virtual-only
initial pose error, does `pose_gain` improve final truth error relative to
matched baselines?

## Scope and safeguards

- Pure virtual cuboid sensor only; no camera, robot driver, gripper, IO or
  trajectory execution.
- Ground truth remains the identity `T_base_model` used only by the virtual
  evaluator.  It must not be used for a real run.
- The injected initial bias is recorded in every `episode.csv` and in
  `config.yaml`; default is zero and hardware-facing launches leave it zero.

## Fixed design

| Item | Value |
|---|---|
| strategies | `fixed_order`, `random_reachable`, `pose_gain` |
| scenes | none, light, heavy deterministic occlusion |
| matched seeds | at least 20 per strategy and scene |
| sensor noise | deterministic 1 mm depth noise, seed recorded per episode |
| initial condition | 15 mm seed-derived translation bias; declared 30 mm prior translation std |
| budget | 6 views maximum; identical candidate set and initial camera pose |
| primary endpoint | episode-level final virtual truth translation error (m) |
| secondary endpoint | initial-to-final virtual truth-error reduction (m) |
| calibration check | predicted versus observed covariance-std reduction, separately from the primary endpoint |

## Decision rule

`pose_gain` is supported in this virtual benchmark only if it has a positive
paired median final-error improvement over both baselines in the pooled
matched-seed analysis, bootstrap 95% confidence intervals exclude zero, and
the effect does not reverse in heavy occlusion.  Otherwise report the effect
as unsupported.  This rule does not establish real-camera or hardware
performance.

## Execution interface

Use the existing batch script with these explicit virtual-only values:

```bash
CS625_REQUIRE_CONTROLLERS=false \
CS625_VIRTUAL_INITIAL_TRANSLATION_BIAS_M=0.015 \
CS625_VIRTUAL_INITIAL_COVARIANCE_STD_M=0.030 \
CS625_DEPTH_NOISE_STD_M=0.001
```

The batch manifest and per-step CSV must contain the two initial-condition
values before a P6 result is considered admissible.
