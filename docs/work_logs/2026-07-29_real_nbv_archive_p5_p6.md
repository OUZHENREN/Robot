# Real-NBV archive update — P5/P6 source snapshot (2026-07-29)

This commit archives the P5 pose-aware virtual observation repair and the P6
controlled virtual error-reduction benchmark interface into
`experiment/real-nbv`.

## Provenance

- source main commit: `8e38ebae8e32a5dc5964af930690b2378df05c3d`
- P5 calibration: virtual visibility/covariance calibration passed
- P5 exploratory prediction-to-next-error correlation: unsupported; it is not
  a PoseGain or strategy-effect result
- P6 smoke run: controller-free virtual cuboid only, with a logged 15 mm
  initial bias and 30 mm scalar prior standard deviation

## Safety and evidentiary boundary

This branch is a code archive for later laboratory preparation.  No real robot,
camera, gripper, IO, trajectory, real object, or real performance result has
been executed or claimed.  Before any lab execution, use the P6 preregistration
and preserve the recorded model/parameter provenance.
