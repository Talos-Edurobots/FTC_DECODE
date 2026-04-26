# Multi-Motor Mechanisms Plan

## Goal

Add a mechanism-level abstraction where the program computes one control output and applies that output to an array of motors.

Target flow:

`subsystem -> mechanism controller logic -> single computed power -> apply to all motors in the mechanism`

This should replace subsystem-local ad hoc syncing like the current `Shooter` pattern, where `motor1` computes power and `motor2` manually mirrors it.

## Current State

The codebase is still primarily built around single-motor abstractions:

- `MotorConfig` wraps one `DcMotorEx`
- `MetaMotor` wraps one `DcMotorEx`
- subsystems usually own one motor object
- `Shooter` is the only clear multi-motor example, and it is implemented manually

Current shooter behavior:

1. `motor1` runs the velocity controller
2. the subsystem reads `motor1.getPower()`
3. that same power is copied into `motor2`

That proves the desired behavior already exists conceptually, but it lives in subsystem code instead of a reusable mechanism abstraction.

## Design Direction

The new feature should be introduced at the mechanism layer, not by making every controller class understand arrays of motors.

Reason:

- the control math should still run once
- only output application should fan out
- each follower motor may still need its own hardware name, direction, and safety settings
- subsystem code should not need to remember to manually mirror power

## Core Principle

Separate these two responsibilities:

### 1. Compute output once

Controllers such as:

- velocity PIDF
- position PIDF
- profiled position control
- open-loop commands

should still produce one power command for the mechanism.

### 2. Apply output many times

A multi-motor mechanism should take that one command and apply it to every motor in its internal motor list.

That keeps the behavior deterministic and avoids duplicated control loops fighting each other.

## Recommended Abstractions

### `MetaMotor`

Keep `MetaMotor` as the single-motor hardware adapter.

Responsibilities:

- bind one FTC motor from `HardwareMap`
- apply clipped power
- expose encoder, velocity, current, mode, and overcurrent data

No multi-motor logic should go directly into `MetaMotor`.

### `MotorGroup` or `MultiMotorMechanism`

Add a new class that owns a collection of `MetaMotor` instances.

Suggested responsibilities:

- initialize all motors
- apply one power value to all motors
- optionally set the same FTC run mode on all motors
- expose the primary motor for feedback reads
- optionally expose aggregate telemetry helpers

Suggested shape:

```java
public final class MotorGroup {
    private final List<MetaMotor> motors;
    private final MetaMotor primaryMotor;

    public void init(HardwareMap hardwareMap) { ... }
    public void setPower(double power) { ... }
    public double getPower() { ... }
    public MetaMotor getPrimaryMotor() { ... }
}
```

Important design rule:

- one motor should be designated as the feedback source

That avoids ambiguity for:

- encoder position
- measured velocity
- current closed-loop error

For symmetric mechanisms like the shooter, reading feedback from one primary motor is enough for the first version.

## Where The Control Logic Should Live

Do not move PID/PIDF math into `MotorGroup`.

Instead:

- controllers compute one output
- the motor-group abstraction applies that output to all motors

This can be done in two reasonable ways.

### Option A: Group only the hardware output

Create a reusable `MotorGroup` and let subsystem or facade code do:

```java
double power = controller.calculate(...);
motorGroup.setPower(power);
```

Pros:

- minimal change
- fits the current architecture
- easy to adopt incrementally

Cons:

- subsystems still know too much about control plumbing

### Option B: Build controlled multi-motor facades

After `MotorGroup` exists, add focused mechanism facades such as:

- `VelocityControlledMotorGroup`
- `ProfiledPositionMotorGroup`
- `OpenLoopMotorGroup`

Pros:

- cleaner subsystem APIs
- matches the direction already described in `MOTORCONFIG_REWRITE_PLAN.md`

Cons:

- slightly larger first implementation

## Recommended Implementation Path

I would do this in two phases.

### Phase 1: Introduce the shared output abstraction

Build a small `MotorGroup` first.

It should support:

- `List<MetaMotor>` or `MetaMotor[]`
- `primaryMotor`
- `init(HardwareMap)`
- `setPower(double)`
- `setMode(RunMode)`
- `stop()`
- `getPrimaryMotor()`
- `getMotors()`

This phase solves the exact feature request:

- calculate one power
- apply it to a motor array

### Phase 2: Move closed-loop mechanisms onto the abstraction

Once `MotorGroup` exists, migrate mechanisms that currently sync followers manually.

Start with:

1. `Shooter`

Later candidates:

2. dual-motor hang
3. linked lift/slides
4. any future mirrored arm or intake design

## First Migration Target: `Shooter`

`Shooter` is the cleanest first target because it already behaves like a multi-motor mechanism.

Current code:

- `motor1` is the leader
- `motor2` is the follower
- `syncSecondaryMotorPower()` mirrors leader power

Planned rewrite:

1. define two motor hardware configs
2. create a `MotorGroup` for the shooter motors
3. use the primary motor for velocity feedback
4. run the velocity controller once
5. send the computed power to the group

Conceptually:

```java
double power = velocityController.calculate(targetVelocity, measuredVelocity, loopState);
shooterGroup.setPower(power);
```

Benefits:

- no manual mirror helper
- no risk of forgetting to sync the second motor
- mechanism behavior becomes reusable

## Interaction With `MotorConfig`

There are two possible paths.

### Short-term path

Keep using `MotorConfig` for now, but add a separate grouping layer above it.

Example:

- `MotorGroup<MotorConfig>` or a group class specialized for `MotorConfig`

This minimizes change, but it extends the lifetime of the legacy abstraction.

### Better long-term path

Use `MetaMotor` as the grouped hardware primitive and keep controllers/facades separate.

That aligns with the existing rewrite direction in:

- `MetaMotor.java`
- `MotorController`
- `PIDFFVelocityController`
- `PIDFFPositionController`
- `TrapezoidalMotionProfileController`

Recommendation:

- if you want the fastest feature delivery, group current motor wrappers first
- if you want the cleanest architecture, group `MetaMotor` and migrate the shooter onto that path

## Configuration Changes

The constants layer needs a mechanism-level config concept.

Today:

- `SHOOTER_CONFIG`
- `SHOOTER2_CONFIG`

Planned direction:

```java
public static List<MetaMotor> SHOOTER_MOTORS = ...
```

or a builder/config object such as:

```java
public static MotorGroupConfig SHOOTER_GROUP = ...
```

That config should define:

- ordered motor members
- which motor is primary
- per-motor direction
- per-motor zero power behavior
- per-motor current alerts
- optional shared max power

## API Expectations

Subsystem-facing APIs should become simple.

Open-loop example:

```java
intakeGroup.setPower(commandedPower);
```

Closed-loop example:

```java
shooterGroup.setTargetVelocity(targetVelocity);
shooterGroup.update(loopState);
```

The subsystem should describe intent.
The group/facade should handle applying one output to many motors.

## Edge Cases To Decide Early

These decisions matter before implementation:

### Motor directions

Some grouped mechanisms need opposite motor directions.

Support this per motor in the config instead of trying to negate power in subsystem code.

### Feedback source

Use one designated primary motor initially.

Later, if needed, you can add:

- average velocity
- averaged position
- health checks across all motors

But that should not block the first version.

### Safety behavior

Decide how the group reacts if one motor is overcurrent:

- stop all motors
- report only
- stop only the faulted motor

For the first version, I would keep it simple:

- expose per-motor health
- let the subsystem choose policy

### Mixed capabilities

Do not allow a group to mix unrelated motors with different mechanism purposes.

A motor group should represent one physical mechanism, not a convenience list.

## Testing Plan

### Unit-level behavior

If local tests are added later, verify:

- one `setPower()` call reaches every motor
- primary motor is used for feedback reads
- per-motor direction is preserved by underlying hardware config
- `stop()` zeros every motor

### Subsystem regression checks

For shooter migration, verify:

- both shooter motors spin together
- velocity telemetry still comes from the intended primary motor
- impact detection still works
- stop/float behavior still affects both motors

### Driver-station validation

On robot hardware, verify:

- both motors initialize correctly
- both motors receive the same magnitude command
- current draw can still be inspected independently if needed
- disabling shooter stops both motors immediately

## Suggested File Order

If I were implementing this next, I would touch files in this order:

1. add `MotorGroup.java` under the `motor` package
2. add a mechanism/group config in `RobotConstants.java`
3. migrate `Shooter.java` to use the group abstraction
4. remove `syncSecondaryMotorPower()` from `Shooter`
5. optionally add telemetry helpers for grouped motors

## Non-Goals For Version 1

To keep scope healthy, version 1 should not try to solve:

- independent closed-loop control per motor
- synchronization by averaging multiple encoders
- automatic fault recovery
- full replacement of `MotorConfig`
- every subsystem migration at once

Version 1 only needs to guarantee:

- compute one output once
- apply it reliably to a collection of motors

## Recommendation

Implement a small `MotorGroup` abstraction first, with one primary feedback motor and fan-out `setPower(double)`.

Then migrate `Shooter` to prove the pattern.

That gives you the requested multi-motor mechanism feature quickly, while also moving the codebase toward the cleaner motor architecture already outlined in `MOTORCONFIG_REWRITE_PLAN.md`.
