# Replacing `MotorConfig`: What I Would Do

If I were in your position, I would not try to "clean up" `MotorConfig` by making a second giant class with better naming. I would turn it into a small composition root and move each responsibility into a dedicated type with one reason to change.

Right now `MotorConfig` mixes:

- hardware binding
- unit conversion
- loop state (`dt`, battery voltage)
- target storage
- control mode selection
- motion profiling
- PID/PIDF math
- safety limits
- telemetry/debugging
- raw motor access

That is why it feels hard to reason about. The package already contains the beginnings of a better design:

- `MotionState`
- `LoopState`
- `MetaMotor`
- `MotorController`
- `PIDFFPositionController`
- `PIDFFVelocityController`
- `TrapezoidalMotionProfileController`
- coefficient classes
- unit classes

My goal would be to finish that design instead of extending `MotorConfig`.

## The Main Idea

I would build a modular motor stack with this flow:

`subsystem -> motor facade -> target/state objects -> controller -> power output -> DcMotorEx`

In other words:

1. A subsystem like `Shooter` or `Turret` should express intent.
2. A small motor facade should gather sensor data and current loop state.
3. A controller should compute output from `reference` and `current`.
4. A hardware adapter should apply power/velocity/position commands safely.

`MotorConfig` currently does all four jobs at once.

## What I Would Keep

I would keep and expand these classes:

- `MotionState`
- `LoopState`
- `MotorController`
- `PIDFFPositionController`
- `PIDFFVelocityController`
- `TrapezoidalMotionProfileController`
- `PIDFFCoefficients`
- `MotionProfilingCoefficients`
- the unit classes under `math.units`

Those are already close to the right abstraction level.

## What I Would Change First

Before migrating subsystems, I would fix the architecture in the motor package itself.

### 1. Make `MetaMotor` the hardware adapter

Today `MetaMotor` only initializes the motor. I would make it own all raw FTC hardware access:

- `init(HardwareMap)`
- `setPower(double)`
- `getPower()`
- `getCurrentPositionTicks()`
- `getVelocityTicksPerSecond()`
- `getCurrentAmps()`
- `setZeroPowerBehavior(...)`
- `setDirection(...)`
- `isOverCurrent()`
- `setCurrentAlert(...)`

This class should be the only place that directly talks to `DcMotorEx`.

That gives you one clean boundary between FTC SDK hardware and your control logic.

### 2. Keep conversion motor-centric

You already have `EncoderConverter`, and I would keep it, but I would keep its scope narrow.

I would make motor-side conversion explicit and central:

- ticks <-> angle
- ticks/sec <-> angular velocity

Right now `MotorConfig` mixes motor conversion with mechanism semantics. That should disappear.

If I were you, I would define a strict boundary:

- the `motor` package owns motor shaft units
- subsystems own mechanism units

That means `EncoderConverter` should only convert:

- motor ticks <-> motor shaft angle
- motor ticks/sec <-> motor shaft angular velocity

It should not know about:

- gear ratio
- spool radius
- slider extension
- linkage geometry
- turret output angle

Those belong in the subsystem layer, because they describe the robot mechanism, not the motor itself.

### 3. Replace static loop globals with injected loop context

`MotorConfig.setDt(...)` and `MotorConfig.setBatteryVoltage(...)` are two of the biggest design smells in the class.

I would remove static shared loop state entirely.

Instead, every update should receive a `LoopState` or direct parameters:

- `dt`
- battery voltage or battery voltage factor

That means each motor update becomes deterministic and testable.

Example direction:

```java
public void update(LoopState loopState) { ... }
```

or

```java
public double calculate(MotionState target, MotionState current, LoopState loopState) { ... }
```

## The New Package Structure I Would Aim For

If I were reorganizing this package, I would end up with something like this:

```text
motor/
  hardware/
    MetaMotor.java
    MotorHardwareState.java
  control/
    MotorController.java
    PIDFFPositionController.java
    PIDFFVelocityController.java
    TrapezoidalMotionProfileController.java
  config/
    PIDFFCoefficients.java
    MotionProfilingCoefficients.java
    MotorLimits.java
    MotorSettings.java
  state/
    MotionState.java
    LoopState.java
    MotorTarget.java
  units/
    Angle.java
    AngularVelocity.java
    EncoderConverter.java
  facade/
    ControlledMotor.java
    VelocityMotor.java
    PositionMotor.java
    ProfiledMotor.java
```

I would not force one facade type to support every mode. I would prefer small facades per use case.

Above the motor package, I would expect mechanism-specific conversion to live in subsystems or subsystem-local helper classes.

## The Facades I Would Actually Build

This is the part I think matters most.

`MotorConfig` is used everywhere because it is convenient, not because it is a good abstraction. To replace it cleanly, I would give subsystems small purpose-built classes instead of one universal one.

### Option A: One generic facade

Example:

- `ControlledMotor`

Responsibilities:

- owns `MetaMotor`
- owns motor-side converter
- owns selected controller
- stores target
- reads current state
- applies output

This is flexible, but it can become a new `MotorConfig` if you are not strict.

### Option B: Multiple focused facades

This is what I would personally do.

Build:

- `OpenLoopMotor`
- `VelocityControlledMotor`
- `PositionControlledMotor`
- `ProfiledPositionMotor`

Each facade would expose only the API relevant to its job.

Examples:

```java
shooterMotor.setTargetVelocity(...)
shooterMotor.update(loopState)
```

```java
turretMotor.setTargetAngle(...)
turretMotor.update(loopState)
```

This prevents illegal combinations and makes each subsystem easier to read.

## How I Would Map Current `MotorConfig` Responsibilities

### Hardware

Move from `MotorConfig` to:

- `MetaMotor`

### Conversion

Split the current conversion responsibility into two levels:

- motor package:
  - `EncoderConverter`
  - motor ticks <-> motor shaft units
- subsystem package:
  - turret gearing
  - slider distance mapping
  - spool / pulley / leadscrew math
  - any mechanism-space target conversion

Rule I would use:

- if it describes the motor axle, it belongs in `motor`
- if it describes the mechanism driven by the motor, it belongs in the subsystem

### Target state

Move from fields like:

- `targetPositionTicks`
- `targetVelocityTicks`
- `xRef`
- `vRef`
- `aRef`

To:

- dedicated target/state holder objects
- controller-internal state where appropriate

Important rule:

- profile internal state should live in the profile controller, not in the facade

You already started doing that in `TrapezoidalMotionProfileController`, which is the right direction.

### PID and feedforward gains

Keep in:

- `PIDFFCoefficients`
- `MotionProfilingCoefficients`

I would also add:

- `MotorLimits`

For motor-side concerns such as:

- max power
- current alert
- maybe ramp rate

I would be careful with angle limits. If the limit is a mechanism limit, I would keep it outside the generic motor layer.

### Safety limits

Move out of the controller math and into the output/application layer.

For example:

- clipping
- current protection
- preventing output beyond a motor-native safety rule

Mechanism limits such as:

- turret min/max angle
- slider min/max extension
- lift top/bottom travel

should usually live above the motor package, because they depend on mechanism meaning.

These are not controller responsibilities.

I would put them in something like:

- `MotorSafety`
- or inside the relevant facade if you want fewer classes

### Telemetry

I would completely remove telemetry from low-level controller classes and from the core motor model.

Reason:

- telemetry is a consumer of state, not part of control logic

Instead, expose state cleanly and let subsystems or debug tools publish it.

For example, let `Debugger` ask for:

- target
- current state
- controller output
- error

But do not let `PIDFFVelocityController` or `MetaMotor` depend on `PanelsTelemetry`.

## Concrete Problems I Would Fix During the Rewrite

These are the things I would actively avoid carrying over.

### 1. Static global loop state

`dt` and battery voltage should not be static shared variables.

### 2. Universal mutable mode switching

`MotorMode` inside a universal motor object is convenient, but it creates invalid states and branching logic everywhere.

I would prefer separate classes over a giant `update()` switch.

### 3. Mixed units

`MotorConfig` stores some things in ticks, some in radians, some in "API only" radians, and mixes motor-space with mechanism-space conversions.

I would choose one internal unit system.

If I were you:

- internal control math in radians and rad/s
- conversion to/from ticks only at the hardware boundary
- mechanism conversion only in the subsystem layer

That makes the controller classes much more coherent, and it matches your newer `MotionState` design.

### 4. Telemetry inside core logic

That creates coupling and makes testing annoying.

### 5. Reusing the same error/integral fields for different behaviors

For example, `lastVelocityError` and `velocityIntegral` currently support multiple methods in one class.

Each controller should own its own integrator and derivative history.

### 6. Facade and controller logic being mixed together

The class that computes output should not also decide FTC run mode initialization, unit conversion, and target clipping.

## The Rewrite Strategy I Would Use

I would do this incrementally, not as a big-bang replacement.

### Phase 1: Finish the low-level building blocks

Implement or improve:

- `MetaMotor`
- motor-centric `EncoderConverter`
- `MotorLimits`
- a clean `LoopState`

At this phase, I would not touch subsystems yet.

### Phase 2: Create one new facade and prove it on one subsystem

I would start with the shooter, because it is conceptually simpler than the turret.

Build:

- `VelocityControlledMotor`

It should:

- own `MetaMotor`
- own `PIDFFVelocityController`
- store velocity target
- read current velocity from hardware
- compute output
- apply clipped power

Then update `Shooter` to use it.

This gives you a real migration pattern.

### Phase 3: Build a profiled position motor for turret/arm style mechanisms

Build:

- `ProfiledPositionMotor`

It should compose:

- `MetaMotor`
- `EncoderConverter`
- `TrapezoidalMotionProfileController`
- safety/limits

Then migrate turret.

This is where you solve:

- angle limits
- hold power / gravity bias if needed
- error tolerance

Important detail:

- `ProfiledPositionMotor` should control motor-shaft position
- the turret subsystem should convert turret angle into motor angle before calling it

### Phase 4: Keep `MotorConfig` as a deprecated adapter temporarily

I would not delete `MotorConfig` immediately.

Instead, for a short time I would either:

- leave it as-is while migrating call sites, or
- rewrite it as a thin adapter over the new classes

If you choose the adapter path, `MotorConfig` becomes a compatibility layer instead of the real implementation.

That lets the rest of the codebase move gradually.

### Phase 5: Remove `MotorMode` branching from subsystem code

Once enough subsystems have migrated, the subsystem code gets simpler:

- no generic `update()` switch
- no "if this mode do X else do Y"
- no hidden motor behavior

Each subsystem just owns the right motor abstraction.

## The API Shape I Would Want

If I were designing the end state, I would want subsystem-facing APIs that read like this:

```java
shooter.setTargetVelocity(AngularVelocity.fromRpm(5000));
shooter.update(loopState);
```

```java
turret.setTargetAngle(Angle.fromDegrees(35));
turret.update(loopState);
```

```java
double velocity = shooter.getMeasuredVelocity().toRadPerSec();
boolean atTarget = shooter.atTarget();
```

That is much clearer than calling a shared class that internally tries to act like four different systems.

Internally, I would want that to mean:

1. subsystem converts mechanism target to motor-shaft target
2. motor facade runs motor-side control only
3. subsystem converts measured motor state back to mechanism state only if needed

## Dynamic Mode Switching

Yes, I think dynamic switching is completely reasonable, especially for mechanisms like the turret.

For example, a turret often needs:

- open-loop control during manual driving
- profiled position control during automatic aiming or preset moves

I would support that, but I would not model it as "swap random controllers everywhere" inside one giant motor class.

Instead, I would make the subsystem or a small facade own the control mode explicitly.

Example direction:

```java
enum TurretControlMode {
    OPEN_LOOP,
    PROFILED_POSITION
}
```

Then the update flow becomes:

- if mode is `OPEN_LOOP`, apply manual power directly
- if mode is `PROFILED_POSITION`, run the profile/controller pipeline

That is cleaner than pretending open loop and motion profiling are the same type of thing.

Important distinction:

- `PIDFFPositionController`, `PIDFFVelocityController`, and `TrapezoidalMotionProfileController` are controller/math strategies
- open loop is just direct output behavior

So I would let the subsystem switch between behaviors, not try to force every behavior into the same abstraction.

### What I Would Do For Turret

If I were implementing the turret rewrite, I would likely give it:

- a control mode enum
- one manual power target
- one profiled position target

Then something conceptually like:

```java
if (mode == OPEN_LOOP) {
    metaMotor.setPower(manualPower);
} else if (mode == PROFILED_POSITION) {
    profiledMotor.setTargetAngle(motorTargetAngle);
    profiledMotor.update(loopState);
}
```

### What To Reset When Switching Modes

This part matters a lot.

When changing from one mode to another, I would reset state that should not carry over:

- controller integrators
- derivative history if needed
- motion profile internal state
- old targets that are no longer valid

For example, when switching from open loop to profiled position:

- capture the current measured position as the profile start state
- reset the profile/controller state
- set the new target

When switching from profiled mode back to open loop:

- stop applying controller output immediately
- clear or ignore the old profiled target until that mode is re-entered

### Design Rule I Would Use

I would allow dynamic switching between control behaviors, but I would keep these rules:

- subsystem chooses the behavior
- motor package computes motor-side control
- open loop stays simple and direct
- closed-loop controllers own only their own internal state

That gives you flexibility without rebuilding `MotorConfig` in disguise.

## What I Would Not Do

I would not:

- add more modes to `MotorConfig`
- keep static `dt` / battery fields
- keep telemetry in controller classes
- keep one class responsible for both profile generation and hardware access
- rewrite every subsystem at once

## My Recommended Order

If you want the shortest path with the least chaos, I would do it in this order:

1. Upgrade `MetaMotor` into the real hardware adapter.
2. Make conversion and loop state explicit.
3. Create `VelocityControlledMotor`.
4. Migrate `Shooter`.
5. Create `ProfiledPositionMotor`.
6. Migrate `Turret`.
7. Decide whether `Intake` really needs a controller or only open-loop access.
8. Turn `MotorConfig` into a compatibility shim.
9. Delete `MotorConfig` only after no subsystem depends on it.

## If I Had To Summarize It In One Sentence

I would replace `MotorConfig` with composition, not inheritance: one hardware adapter, one controller per behavior, one small motor-shaft facade per use case, and mechanism conversion kept outside the motor package.

## First Practical Step

If I were starting today, I would implement `VelocityControlledMotor` first and migrate `Shooter` before touching anything else. That will tell you very quickly whether the new architecture is genuinely cleaner or just theoretically cleaner.
