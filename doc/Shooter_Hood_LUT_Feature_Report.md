# Shooter and Hood Interpolated LUT Feature Report

## Goal

Add two calibrated lookup tables for scoring shots from robot field position:

- shooter LUT: robot distance from the blue goal at `(x=15, y=128)` -> shooter target velocity in ticks per second
- hood LUT: robot distance from the same goal plus shooter target velocity -> hood PWM servo position from `0.0` to `0.5`

The feature also adds a `Debugger.java` TeleOp entry for testing the tables and adding calibration pairs during tuning.

## Implementation Plan

1. Create a one-dimensional interpolated LUT for shooter velocity.
2. Create a two-dimensional interpolated LUT for hood position.
3. Add a shared holder for the blue-goal coordinate, starter samples, and distance helpers.
4. Add a Debugger TeleOp mode that:
   - computes distance from `(15, 128)` using either a test robot pose or a manual distance
   - displays predicted shooter velocity and hood position
   - allows adding/updating shooter and hood calibration samples
   - optionally applies the predicted outputs to the real shooter motor and hood servo

## Files Added

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/ShooterVelocityLut.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/HoodAngleLut.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/ShooterHoodLuts.java`

## Files Updated

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/Debugger.java`

## Shooter LUT

`ShooterVelocityLut` stores samples shaped like:

```text
distance_from_goal -> target_velocity_ticks_per_second
```

Runtime behavior:

- samples are sorted by distance
- exact distance matches return the stored target velocity
- distances between two samples use linear interpolation
- distances outside the calibrated range clamp to the nearest endpoint
- adding a sample at an existing distance replaces that sample

## Hood LUT

`HoodAngleLut` stores samples shaped like:

```text
distance_from_goal, shooter_velocity_ticks_per_second -> hood_servo_position
```

Runtime behavior:

- exact distance/velocity matches return the stored hood position
- nearby samples are blended with inverse-distance weighting
- distance and velocity axes are normalized by the current sample spread before weighting
- hood outputs are clipped to the valid servo range `0.0` through `0.5`
- adding a sample at an existing distance/velocity pair replaces that sample

## Shared LUT Holder

`ShooterHoodLuts` defines:

```text
BLUE_GOAL_X = 15.0
BLUE_GOAL_Y = 128.0
```

It also contains starter calibration samples. These are not final shot data; they are seed values so interpolation and the debug OpMode work immediately. Replace or extend them with real shot-tested calibration points.

## Debugger TeleOp

The new mode appears under:

```text
Debugger -> high level -> shooter + hood lut debug
```

Primary controls:

- `A`: add/update shooter sample using current distance and `testShooterVelocity`
- `B`: add/update hood sample using current distance, current hood velocity input, and `testHoodPosition`
- `X`: toggle distance source between test robot pose and manual distance
- `Y`: toggle hood velocity input between predicted shooter velocity and manual test velocity
- `LB`: toggle shooter motor run state
- `RB`: toggle whether LUT outputs are applied to hardware
- left stick: adjust robot pose, or manual distance when manual distance mode is selected
- right stick Y: adjust manual test shooter velocity
- triggers: adjust manual test hood position

Panels-configurable fields are exposed for faster tuning from the driver station.

## Tuning Workflow

1. Open `shooter + hood lut debug`.
2. Put the robot at a known field position or set a manual distance.
3. Adjust `testShooterVelocity` until shots are consistent.
4. Press `A` to save the shooter distance/velocity pair.
5. Adjust `testHoodPosition` for that same shot setup.
6. Press `B` to save the hood distance/velocity/position pair.
7. Repeat across the useful scoring area.
8. Move final tuned values into `ShooterHoodLuts` so they are present after robot restart.

## Notes

- The current runtime code does not automatically replace match TeleOp's close/far shooter presets. The LUTs are ready and reusable, but match integration should happen after enough shot data is collected.
- Blue goal distance is implemented exactly as requested from `(15, 128)`.
- Red alliance support can be added later with either a mirrored goal coordinate or separate red calibration samples.
