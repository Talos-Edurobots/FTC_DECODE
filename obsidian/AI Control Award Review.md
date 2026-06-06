# Current Control Award review

## Verdict

The portfolio describes the final competition architecture that will be submitted:

```text
Pinpoint pose + fixed Limelight AprilTag pose -> validated relocalization
corrected pose -> turret virtual aim-point LUT
corrected pose -> shooter velocity LUT
corrected pose + measured velocity -> hood LUT
encoder/current/sensor feedback -> controllers, transfer automation, LEDs, telemetry
```

## Final submitted system

- Limelight is fixed above the intake, outside the rotating turret.
- Validated AprilTag observations correct accumulated Pinpoint odometry drift.
- Invalid, ambiguous, stale, or implausibly large pose corrections are rejected.
- Pinpoint remains the continuous pose source when no acceptable tag observation is available.
- Turret position LUT is enabled by default and has red samples mirrored for blue.
- Shooter target velocity is linearly interpolated from goal distance.
- Hood position is interpolated from goal distance and measured shooter velocity.
- Turret movement uses a trapezoidal profile and PIDFF with encoder feedback.
- Shooter uses encoder velocity PIDFF and battery-voltage scaling.
- Transfer stops collection after debounced three-sensor fullness or intake overcurrent.
- Competition/debug telemetry modes, loop statistics, and throttled current readings are implemented.
- Shoot-on-the-move remains disabled.

## Code issue to fix before alliance testing

`ShooterHoodLuts.distanceToGoal(robotPose, isRed)` currently ignores `isRed` and always measures from `BLUE_GOAL_POSE`. The turret has separate red/blue targets, but shooter velocity and hood distance do not. Confirm the coordinate convention and add the correct mirrored/red goal before presenting both-alliance LUT coverage.

## Claims that need careful wording

- Feeding is driver-commanded; it is not blocked automatically while the shooter is busy.
- Relocalization quality must be shown with pose-error data, not only a camera screenshot.
- State the exact observation rejection rules used in the final implementation.
- Simulator coverage is strong comparative evidence, but physical spot-checks are needed before claiming field reliability.
- The simulator currently supports interactive testing and LUT export; automated grid sweeps and heatmaps are still to be added or recorded externally.

## Best LUT evidence

Use before/after scoreable field coverage rather than fixed shot counts at a few points. Compare the previous fixed target and two shot pairs against all three enabled LUTs using the same simulator grid. Show two maps, the added scoreable region, boundary failures, and several physical spot-checks.

This is clearer for judges because it directly answers the match question: **From how much of the field can the robot score?**

## Remaining evidence

1. Fix or verify alliance-specific shooter/hood goal distance.
2. Validate fixed-Limelight pose relocalization under match movement and lighting.
3. Measure pose error before and after correction at known field points.
4. Record accepted/rejected observations and rejection reasons.
5. Before/after scoreable-area maps and coverage increase.
6. Physical validation of representative and boundary simulator poses.
7. Transfer reliability trials.
8. Competition/debug loop-rate results.
9. Timed turret data only if numerical settle-time or accuracy claims are included.

## Three-page portfolio balance

| Page | Main content |
| --- | --- |
| 1 | Driver automation and corrected-pose scoring architecture |
| 2 | Limelight relocalization plus adaptive turret/shooter/hood control |
| 3 | Scoreable field coverage, localization validation, and measured results |

The development apps are worth including as a small toolchain row. Write what each enabled: Android Studio for implementation/debugging, Panels for live tuning/graphs, Google Colab/Python for analysis, Pedro Pathing for pose/path following, and the custom simulator for LUT calibration. Do not spend a full portfolio block on software names.
