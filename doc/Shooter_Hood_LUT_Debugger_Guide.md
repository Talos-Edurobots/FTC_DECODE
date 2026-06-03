# Shooter + Hood LUT Debugger Guide

`ShooterHoodLutDebug` is a tuning mode inside `Debugger`.

Its purpose is to let you stand the robot at a known field position, tune the shooter velocity and hood servo position until the shot is good, then press a button to print LUT-ready sample lines into Logcat.

The mode is available from:

```text
Debugger -> high level -> shooter + hood lut debug
```

## What It Tunes

The shooter velocity LUT is one-dimensional:

```java
ShooterVelocityLut.sample(distanceFromGoal, targetVelocityTicksPerSecond)
```

The hood LUT is two-dimensional:

```java
HoodAngleLut.sample(distanceFromGoal, shooterVelocityTicksPerSecond, hoodServoPosition)
```

That means every good shot produces two useful calibration lines:

```java
ShooterVelocityLut.sample(distance, velocity),
HoodAngleLut.sample(distance, velocity, hood),
```

Those lines are logged with `Log.d()` when `A` is pressed.

## Panels Configurables

### `robotX`

The robot's X position on the field, in inches.

The debugger uses `robotX` and `robotY` to calculate distance from the robot to the goal. That calculated distance is what gets logged into both LUT sample lines.

### `robotY`

The robot's Y position on the field, in inches.

Together with `robotX`, this determines:

```java
distanceFromGoal = distance(robotX, robotY, goalX, goalY)
```

### `targetVelocityTicksPerSecond`

The shooter velocity being tested.

When `runShooter` is enabled, the shooter motor is commanded to this value. When a sample is logged, this value is used in the shooter velocity LUT line:

```java
ShooterVelocityLut.sample(distanceFromGoal, targetVelocityTicksPerSecond)
```

### `hoodServoPosition`

The hood servo position being tested.

This value is clipped from `0.0` to `0.5`, because `Shooter.setHoodAngle()` clips the hood to that range. When a shot is good, this value is logged into the hood LUT line:

```java
HoodAngleLut.sample(distanceFromGoal, velocity, hoodServoPosition)
```

### `isRedAlliance`

Chooses which alliance goal calculation should be used.

Right now, `ShooterHoodLuts.distanceToGoal(...)` uses the blue goal pose internally, but this setting is still included so the tuning mode already has the correct shape if red/blue goal behavior is expanded later.

### `runShooter`

Controls whether the shooter motor is running.

This can be changed from Panels or toggled with `X`.

### `feedTransfer`

Controls whether the transfer feeds rings into the shooter.

This can be changed from Panels or toggled with `Y`.

### `useMeasuredVelocityForHoodLog`

Controls which velocity is written into the hood LUT sample.

When `false`, the hood line uses the target velocity:

```java
HoodAngleLut.sample(distanceFromGoal, targetVelocityTicksPerSecond, hoodServoPosition)
```

When `true`, the hood line uses the actual measured shooter velocity:

```java
HoodAngleLut.sample(distanceFromGoal, shooter.getVelocity(), hoodServoPosition)
```

This matters because hood angle depends on the real projectile speed. If the shooter is commanded to `1400` ticks/sec but is actually spinning at `1360` ticks/sec, the measured value may create a more physically accurate hood sample.

For a simpler LUT, using target velocity is easier to reason about. For a more accurate physical calibration, using measured velocity can be better.

### `hoodNeighborCount`

Controls how many nearby hood LUT samples are used when showing the predicted hood position from the existing LUT.

The hood LUT estimates a hood position by looking at nearby samples in this two-dimensional space:

```text
distanceFromGoal + shooterVelocityTicksPerSecond
```

A lower `hoodNeighborCount` follows the closest samples more tightly.

A higher `hoodNeighborCount` averages more samples and can make the predicted hood value smoother.

This does not change the sample that gets logged. It only affects the telemetry value called `predicted hood from LUT`.

### `poseAdjustInchesPerSecond`

Controls how fast `robotX` and `robotY` move when using the left stick.

Left stick X changes `robotX`.

Left stick Y changes `robotY`.

Higher values make the pose numbers move faster.

### `velocityAdjustTicksPerSecond`

Controls how fast `targetVelocityTicksPerSecond` changes when using the right stick Y axis.

Higher values make velocity tuning faster.

### `hoodAdjustPerSecond`

Controls how fast `hoodServoPosition` changes when using the triggers.

Right trigger increases the hood servo position.

Left trigger decreases the hood servo position.

Higher values make hood adjustment faster.

## Gamepad Controls

`A`: log the current LUT sample to Logcat.

`B`: toggle whether the hood LUT line uses target velocity or measured velocity.

`X`: toggle the shooter motor.

`Y`: toggle transfer feeding.

Left stick: adjust `robotX` and `robotY`.

Right stick Y: adjust `targetVelocityTicksPerSecond`.

Triggers: adjust `hoodServoPosition`.

## Telemetry

The mode shows:

- alliance
- robot X/Y
- distance from goal
- target shooter velocity
- measured shooter velocity
- whether the shooter is ready
- shooter power
- hood servo position
- whether hood logging uses target or measured velocity
- predicted shooter velocity from the existing LUT
- predicted hood position from the existing LUT
- turret target angle
- turret measured angle
- whether the turret is at target
- transfer state
- transfer fullness/current
- the last logged velocity LUT line
- the last logged hood LUT line

## Suggested Workflow

1. Put the robot at a known field position.
2. Set `robotX` and `robotY` in Panels.
3. Press `X` to spin up the shooter.
4. Tune `targetVelocityTicksPerSecond`.
5. Tune `hoodServoPosition`.
6. Press `Y` to feed and test shots.
7. When the shot is correct, press `A`.
8. Copy the printed `ShooterVelocityLut.sample(...)` and `HoodAngleLut.sample(...)` lines from Logcat into `ShooterHoodLuts`.

