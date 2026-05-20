# Programming Department Notebook Draft

This draft is written for the team's larger engineering notebook. It is more technical than the portfolio section and can be expanded with screenshots, diagrams, tuning logs, and test results.

## Programming Overview

The programming goal for DECODE is to make the robot easier to control, more repeatable in autonomous, and more consistent when scoring. Our code is organized around subsystem classes, reusable motor-control facades, Pedro Pathing localization/path following, and a telemetry system that separates match information from debugging information.

The main programming systems are:

- autonomous path following and state control,
- TeleOp driver control,
- pose-based turret aiming,
- shooter velocity regulation,
- transfer and intake sensor logic,
- reusable motor controller abstractions,
- telemetry and loop-performance management.

## Software Architecture

The robot code is split into three main layers.

| Layer | Purpose | Examples |
| --- | --- | --- |
| OpModes and match flow | Decide what the robot should do during autonomous or TeleOp | `MainTeleOp`, `NewAuto`, alliance wrappers |
| Subsystems | Represent mechanisms and their game behavior | `Shooter`, `Turret`, `Transfer`, `Intake`, `Leds`, `HardwareManager` |
| Reusable control utilities | Provide generic motor control, telemetry, units, and loop state | `VelocityControlledMotor`, `ProfiledPositionMotor`, `PIDFFVelocityController`, `TelemetryHub` |

This organization lets mechanism code talk in robot concepts, such as "aim at goal" or "collect artifacts," while the lower-level motor package handles encoder units, PIDF, feedforward, and hardware output.

## Autonomous

Autonomous is handled by `NewAuto`. The routine uses Pedro Pathing and goBILDA Pinpoint localization to follow planned field paths. Paths are built from Bezier lines and Bezier curves, then chained together for repeated scoring and pickup cycles.

The autonomous routine is a state machine. Each state starts a path or mechanism action, then waits for feedback before continuing. Examples of feedback gates include:

- `follower.isBusy()` to wait until path following is complete,
- `shooter.isBusy()` to wait until the shooter is close to target speed,
- timers for transfer/flicker actions,
- transfer state changes for collection and shooting.

This is more reliable than a purely timed auto because the robot can adapt to small differences in motion time. The same autonomous structure supports alliance mirroring, so red and blue paths do not need to be written completely separately.

Key concept: autonomous is not only path movement. It is coordination between drivetrain localization, mechanism readiness, scoring timing, and field strategy.

## TeleOp

`MainTeleOp` coordinates driver controls, localization, turret aiming, shooter presets, transfer states, LEDs, and telemetry.

Important TeleOp behaviors:

- The robot can preserve pose from autonomous through `RobotPoseStorage`.
- Drivers can toggle Limelight aiming or pose-based goal aiming.
- Drivers can switch close/far shot presets, changing shooter target velocity and hood angle.
- Slow mode scales drivetrain input for more precise control.
- LEDs communicate shooter/turret/transfer status to drivers.
- Gamepad controls change telemetry mode for testing.

TeleOp is written so drivers still control strategy, while software handles repeatable low-level tasks like aiming, velocity regulation, sensor stopping, and feedback display.

## Localization and Path Following

The drivetrain uses Pedro Pathing with mecanum constants and a Pinpoint localizer. The localizer tracks field position using odometry pods and configured pod offsets. This pose estimate is used for:

- autonomous path following,
- TeleOp pose tracking,
- turret aiming from field position,
- drawing the robot on the FTC Dashboard/Panels field.

Manual bulk caching is enabled in `HardwareManager`. Each loop clears the hub cache once, allowing repeated encoder and sensor reads in that loop to come from a consistent snapshot. This improves loop consistency and makes the code easier to reason about.

## Turret Aiming

The turret has two main aiming modes.

| Mode | Feedback | Control method | Use case |
| --- | --- | --- | --- |
| Pose-based aiming | Robot pose, heading, turret encoder | Field geometry plus profiled PIDF position control | Normal automatic goal aiming |
| Limelight aiming | Limelight horizontal target offset (`tx`) | Manual PID correction | Vision-based target alignment |

For pose-based aiming, the code computes the angle from the robot to the goal using the robot's field pose. It subtracts robot heading to get the turret angle relative to the robot. That target is clipped inside the turret's mechanical range and converted through the turret gear ratio before being sent to the motor controller.

The turret uses a trapezoidal motion profile. Instead of instantly chasing a target, it creates a smooth reference with limited velocity and acceleration. A PIDF controller then follows that reference. Feedforward terms help overcome friction and support planned motion, while feedback corrects error.

Reliability protections:

- target angle is clipped to mechanical limits,
- motor-side angle accounts for the 2.8 external gear ratio,
- controller state resets when switching between profiled and manual modes,
- overcurrent and current data are exposed through telemetry,
- optional position aim LUT can use field samples to choose virtual aim points.

Lesson learned: moving-shot compensation should be improved. The current code shifts the pose based on velocity and distance. A better model would separate tangential velocity from radial velocity and use projectile flight time to compute an angular lead.

## Shooter Control

The shooter uses encoder velocity feedback through `VelocityControlledMotor`. The target velocity is set from presets, and the measured velocity is compared against the target every loop.

The shooter controller includes:

- PID feedback on velocity error,
- feedforward for expected motor power,
- battery-voltage scaling,
- filtered velocity for smoother signal analysis,
- busy/ready detection based on target error,
- impact detection based on sudden filtered velocity drop.

The shooter has two motors: the main controlled motor and a follower motor that mirrors the main motor power. The hood servo changes shot angle for different presets.

Why this matters: a flywheel shooter depends on speed consistency. Encoder feedback lets the robot correct for battery sag, load changes, and speed loss after shooting.

## Transfer and Intake Logic

The transfer system has three states: `STOP`, `COLLECT`, and `SHOOT`. The state controls the intake and gate together so the mechanism behaves as one coordinated system.

Three distance/color sensors detect artifacts inside the robot. These sensors update at 5 Hz, which is fast enough for collection logic but avoids unnecessary sensor reads every control loop. If all sensors indicate the robot is full, or the intake overcurrent alert triggers, the code stops collection.

This reduces driver workload and protects the mechanism from jams or current spikes.

## Motor Control Package

The motor package exists so each subsystem does not need to reimplement encoder conversion and PIDF logic.

Important abstractions:

- `MetaMotor`: hardware adapter for a motor and its configuration.
- `VelocityControlledMotor`: motor-shaft velocity control for mechanisms like the shooter.
- `ProfiledPositionMotor`: motion-profiled position control for mechanisms like the turret.
- `PIDFFVelocityController`: combines velocity feedback with feedforward.
- `TrapezoidalMotionProfileController`: generates smooth position, velocity, and acceleration references.
- `LoopState`: passes loop `dt` and battery-voltage factor into controllers.
- `EncoderConverter`: converts between ticks, angles, and velocities.

This package makes the code more reusable. When a new mechanism needs controlled motion, we can choose a facade instead of starting from raw motor power.

## Telemetry System

Telemetry was redesigned because publishing every debug value every loop can slow the robot and make driver information hard to read.

The current telemetry design has:

- `TelemetryProvider` for subsystems that expose data,
- `TelemetryCollector` for collecting and filtering fields,
- `TelemetryHub` for publishing at the correct cadence,
- `TelemetryMode` values: `OFF`, `COMPETITION`, `DEBUG`, `TRACE`,
- `TelemetryCostClass` values: `STATIC`, `CHEAP`, `BULK_CACHED`, `NON_BULK`, `FORMATTED`,
- `ThrottledValue` for expensive values such as continuous current readings.

Competition telemetry is intentionally smaller and publishes at a slower interval. Debug telemetry contains more tuning data and can publish every loop when needed.

Key lesson: telemetry is part of the control system because it affects loop time, debugging speed, and driver decision-making.

## Testing and Tuning Process

Recommended process for future tuning:

1. Test each subsystem alone before full integration.
2. Tune shooter velocity PIDF using target velocity, measured velocity, filtered velocity, and recovery time after shots.
3. Tune turret aiming by measuring final angle error from multiple field positions.
4. Test autonomous path chains one state at a time before running the whole routine.
5. Compare loop frequency in competition telemetry and debug telemetry.
6. Record failures in the notebook with cause, attempted fix, result, and next step.

Useful measurements to add:

- shooter spin-up time,
- shooter velocity error,
- turret angle error,
- autonomous completion percentage,
- average and worst loop time,
- intake full-detection reliability.

## Current Strengths

- The robot uses feedback control in both autonomous and TeleOp.
- The scoring system combines pose estimation, turret aiming, and shooter velocity control.
- Autonomous is state-based and waits for real robot conditions.
- Sensor logic protects the intake/transfer system.
- Telemetry is structured by mode and cost, which helps maintain loop consistency.
- The motor package creates reusable abstractions for future mechanisms.

## Planned Improvements

- Replace moving-shot pose offset with a physics-based angular lead model.
- Centralize battery-voltage sampling so shooter and turret use the same cached value.
- Add more measured tuning data for portfolio evidence.
- Continue reducing hot-loop work by avoiding duplicate reads/writes and unnecessary telemetry formatting.
- Add diagrams for autonomous state flow, turret/shooter control flow, and telemetry architecture.

