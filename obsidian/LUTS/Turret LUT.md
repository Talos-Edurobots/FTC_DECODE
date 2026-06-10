# Turret position LUT

The turret LUT does not directly store turret angles. Each sample stores:

```text
robot field position -> calibrated virtual aim point
```

The robot pose comes from Pinpoint odometry corrected by validated AprilTag observations from the fixed Limelight above the intake. At that corrected pose, the code selects the nearest samples and weights them by inverse squared distance. It interpolates a virtual aim point, then field geometry converts that point into the turret angle. Red-side samples are mirrored for the blue alliance.

This corrects the limitation of always aiming at one fixed goal coordinate. It also creates smooth changes between calibrated field positions instead of a list of disconnected presets.

## Validation

The clearest before/after evidence is scoreable field coverage:

1. Test a grid of reachable poses with the old fixed target and two fixed velocity/hood pairs.
2. Mark each pose as scoreable or not scoreable.
3. Repeat with turret, velocity, and hood LUTs enabled.
4. Overlay the maps and compare scoreable area or scoreable grid-cell count.
5. Physically verify representative poses and the edge of the predicted scoring region.
6. Repeat boundary checks after long driving periods to verify that Limelight relocalization preserves LUT alignment when odometry would otherwise drift.

This test matches the real behavior better than reporting an arbitrary number of shots at one point. At a badly calibrated pose, the trajectory misses the goal; at a correctly calibrated pose, normal repeatability should make almost every shot score.

See [[Simulator]] and [[TurretLUT_img.png]].
