# Turret aiming evolution

## V1 - Limelight mounted on the turret

The Limelight measured horizontal target error and a PID controller rotated the turret.

**What worked:** simple implementation and direct target feedback.

**Why we changed it:** limited field of view caused target loss, target visibility was not guaranteed, and slow movement was needed to reduce gear stress. Aiming at one visible tag point also did not produce the correct shot from every field position.

## V2 - Pose aiming and trapezoidal motion profiling

Pedro Pathing pose and field geometry calculate the required turret angle:

$$
\theta_{turret}=atan2(y_{target}-y_{robot},x_{target}-x_{robot})-\theta_{robot}
$$

A trapezoidal profile generates position, velocity, and acceleration references. PIDFF follows those references while limiting acceleration and mechanical stress.

**Improvement:** the turret can aim even when the target is outside the camera view and no longer depends on rotating the whole robot.

**Remaining problem:** one fixed target point did not account for position-dependent shot behavior. Odometry drift can also move the calculated aim away from the real goal.

We prototyped shoot-on-the-move compensation in the [[Simulator]], but it was inconsistent and slowed cycles, so the match call remains disabled.

## V3 - Fixed Limelight relocalization and interpolated position aim LUT

The Limelight is mounted above the intake, outside the turret. Because it no longer rotates with the turret, its camera transform relative to the robot remains fixed. AprilTag observations provide an independent field-pose measurement that corrects accumulated Pinpoint odometry drift.

Vision observations are validated before changing the robot pose. Frames without a valid tag solution, with excessive ambiguity, or with an implausibly large jump are rejected. Accepted observations correct the pose used by the aiming system instead of directly commanding the turret.

The corrected pose feeds a `PositionAimLut` for both alliances. Each sample maps a robot field position to a calibrated virtual aim point. The three nearest samples are weighted by inverse squared distance, and the resulting aim point is converted to a turret angle. Blue-side samples are mirrored from the calibrated red-side set.

This architecture is active in match TeleOp. The driver can still:

- force the turret forward

The Limelight therefore improves every pose-dependent scoring calculation rather than only measuring turret angle.

## Portfolio claim

> A fixed Limelight above the intake corrects Pinpoint odometry drift using validated AprilTag poses. The corrected pose feeds a position-dependent virtual aim-point LUT, field geometry, encoder feedback, and profiled PIDFF turret control.

## Validation

Use a before/after field map:

- **Before:** fixed target point plus two fixed shot combinations.
- **After:** interpolated turret aim point, shooter velocity, and hood position.
- Mark every tested pose as scoreable or not scoreable.
- Report the increase in scoreable area or scoreable grid cells.
- Physically spot-check representative and boundary poses.
- At taped field points, compare pose error before and after accepted Limelight corrections.
- Record rejected observations and their rejection reason.

Timed settle tests are only needed if the portfolio claims a numerical lock time, overshoot, or final angle error.

## Limitations

- AprilTags are not always visible, so Pinpoint odometry continues between corrections.
- Vision observations must be rejected when their quality or pose jump is unsafe.
- Shoot-on-the-move is not a current match feature.
- The simulator alone does not prove real-world reliability.
