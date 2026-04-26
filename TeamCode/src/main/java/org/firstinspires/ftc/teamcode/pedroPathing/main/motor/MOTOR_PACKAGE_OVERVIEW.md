# Motor Package Overview

This document describes the intended shape of the `motor` package after the
`MotorConfig` rewrite work.

## Why This Package Exists

The package owns motor-side control logic:

- FTC motor hardware access
- encoder conversion
- controller math
- loop-time and battery compensation
- motor-shaft state

It does not own mechanism semantics such as turret output angle, linear slide
extension, spool geometry, or linkage math. Those stay in subsystem code.

## Design Goal

Replace the old all-in-one `MotorConfig` abstraction with composition:

`subsystem -> facade -> controller -> MetaMotor -> DcMotorEx`

This makes the code easier to test, migrate, and reason about.

## Main Types

### `MetaMotor`

`MetaMotor` is the FTC SDK boundary.

Responsibilities:

- initialize `DcMotorEx`
- set power
- set mode
- read encoder position
- read velocity
- read current draw
- configure current alert, direction, and zero-power behavior

If code needs raw SDK motor interaction, it should go here rather than into a
controller or subsystem.

### `LoopState`

`LoopState` carries per-update context:

- `dt`
- battery-voltage scaling

The main reason it exists is to avoid static global loop state.

### `MotionState`

`MotionState` is the motor-side state object passed into controllers:

- position
- velocity
- acceleration

The current convention is motor-shaft radians and rad/s inside control logic.

### Controllers

- `PIDFFPositionController`
- `PIDFFVelocityController`
- `TrapezoidalMotionProfileController`

Controllers should only compute control output. They should not know about FTC
hardware, telemetry, or subsystem-specific meaning.

### Facades

- `OpenLoopMotor`
- `VelocityControlledMotor`
- `ProfiledPositionMotor`

A facade owns:

- one hardware adapter
- one converter
- one controller strategy
- target storage relevant to that use case
- the per-loop update path

The point is to make each mechanism own a motor abstraction that matches its
actual job instead of one universal mode-switching class.

## Legacy Compatibility

`MotorConfig` still exists because several subsystems still depend on it.

Current status:

- `Shooter` has been migrated to `VelocityControlledMotor`
- `Turret`, `Intake`, `DriveTrain`, and `Hang` still use legacy paths

That is deliberate. Migration is incremental so behavior can be preserved while
subsystems move one by one.

## Tuning Boundary

Legacy tuning in this codebase was mostly expressed in encoder ticks.

The new controller layer works in radians and rad/s, so
`MotorCoefficientScaler` exists to convert legacy tick-space tuning into the new
motor-space unit system without changing runtime behavior more than necessary.

## Recommended Usage

### Velocity control

Use `VelocityControlledMotor` for mechanisms like the shooter.

Typical loop:

1. Build or inject the facade
2. Call `init(hardwareMap)`
3. Set the target velocity
4. Call `update(loopState)` every loop

### Profiled position control

Use `ProfiledPositionMotor` for mechanisms that move to a position through a
trapezoidal profile.

Typical loop:

1. Build or inject the facade
2. Call `init(hardwareMap)`
3. Set angle limits if needed
4. Set the target angle
5. Call `update(loopState)` every loop

Mechanism-space conversion should happen before calling the motor facade.

## Rules For Future Changes

- Do not add new responsibilities to `MotorConfig`
- Keep telemetry outside controllers
- Keep FTC SDK access inside `MetaMotor`
- Keep mechanism conversion outside the motor package
- Prefer a new small facade over a new `MotorMode`
- Pass loop context explicitly instead of using static globals
