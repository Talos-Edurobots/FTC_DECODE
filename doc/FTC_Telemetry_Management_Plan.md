# FTC Telemetry Management Plan

## Why this needs its own architecture

Your instinct is correct: telemetry is not "just some `addData()` calls".

In this codebase, telemetry is currently mixed across:

- OpModes like `MainTeleOp`
- subsystems like `Turret` and `Leds`
- legacy motor-package code like `MotorConfig`

That creates four problems:

1. hot-loop logic and telemetry logic are coupled
2. expensive values and cheap values are treated the same
3. static values get re-sent every loop
4. there is no clean distinction between competition telemetry and debug telemetry

The result is exactly what you described:

- telemetry is not flexible
- useful things get buried
- static values are repeated
- debugging becomes expensive and messy

This document proposes a telemetry system that is:

- motor-package-centered
- cheap by default
- flexible per OpMode
- extensible by subsystems
- explicit about data cost

## Important correction to the previous optimization report

Your note is important:

> current alerts on `DcMotor`s are bulk-read, while current readings are not

That changes telemetry strategy in a meaningful way.

It means:

- `isOverCurrent()` / current-alert style booleans are relatively cheap and safer to include in competition telemetry
- `getCurrent(CurrentUnit.AMPS)` style continuous current measurements are more expensive and should usually be throttled or debug-only

So from now on, the telemetry architecture should classify:

- **bulk-cached state**
- **non-bulk sensor reads**
- **derived/computed values**

separately.

That is one of the core ideas of this plan.

## Core design principle

The motor package should **expose telemetry-ready state**, but it should **not directly publish telemetry**.

That means:

- controllers do not call `TelemetryManager`
- `MetaMotor` does not call `TelemetryManager`
- facades do not push strings directly
- subsystems and OpModes choose what to publish

So the architecture becomes:

`hardware/controller/facade -> state snapshot -> telemetry selection/policy -> telemetry output`

This keeps telemetry as a consumer of state, not part of control logic.

## What I recommend building

I would split the system into five layers.

## 1. Telemetry source layer

These are the classes that know robot state.

Examples:

- `MetaMotor`
- `VelocityControlledMotor`
- `ProfiledPositionMotor`
- `Shooter`
- `Turret`
- `Intake`
- `Follower` wrapper if you add one

These classes should expose state through methods or small snapshot objects.

They should not decide:

- whether a value belongs in competition telemetry
- how often a value should be shown
- what panel or section it belongs to

## 2. Snapshot layer

This is the most important addition.

Each mechanism should expose a **snapshot** or **telemetry view** object that contains the values telemetry may want.

Example idea:

```java
public final class MotorTelemetrySnapshot {
    public double targetVelocityTicksPerSecond;
    public double measuredVelocityTicksPerSecond;
    public double appliedPower;
    public int positionTicks;
    public boolean overCurrent;
    public boolean busy;
    public double error;
}
```

For profiled motors:

```java
public final class ProfiledMotorTelemetrySnapshot {
    public double targetAngleRad;
    public double measuredAngleRad;
    public double measuredVelocityRadPerSec;
    public double referenceVelocityRadPerSec;
    public double referenceAccelerationRadPerSec2;
    public double appliedPower;
    public boolean overCurrent;
    public boolean atLowerLimit;
    public boolean atUpperLimit;
}
```

These snapshots should contain **data**, not strings.

That keeps:

- formatting outside the motor package
- unit conversion explicit
- telemetry reusable for dashboards, logs, and DS telemetry

## 3. Telemetry policy layer

This layer decides:

- what to show
- when to show it
- in which mode to show it

This is where your "debugging telemetry" vs "competition telemetry" idea should live.

I would model this explicitly with:

- telemetry modes
- value cost classes
- update-rate policies

## 4. Telemetry registry layer

This layer lets subsystems and motor facades all contribute telemetry through one shared architecture.

Instead of each class calling `telemetryM.addData(...)` directly, they register a provider.

Example idea:

```java
telemetryHub.register(shooterTelemetryProvider);
telemetryHub.register(turretTelemetryProvider);
telemetryHub.register(driveTelemetryProvider);
```

Then the OpMode decides per loop:

```java
telemetryHub.publish(competitionMode, telemetry);
```

This solves your goal of:

- motor package owning the structure
- subsystems adding their own values
- one shared architecture

## 5. Output layer

This is the final formatting step:

- Driver Station telemetry
- Panels telemetry
- optional log output

Only this layer should know about `PanelsTelemetry.INSTANCE.getTelemetry()`.

In the implemented version of this architecture, output cadence is also part of this layer:

- `OFF` and `COMPETITION` refresh at `5 Hz`
- `DEBUG` and `TRACE` refresh every loop on Panels
- Driver Station telemetry is capped separately at `10 Hz`

## The telemetry modes I would define

I would not stop at only "debug" and "competition".

I would define four modes.

## 1. `OFF`

Use for maximum performance testing.

Contains:

- nothing except maybe loop-rate and fatal alerts

Purpose:

- throughput testing
- diagnosing whether telemetry itself is slowing the loop

## 2. `COMPETITION`

Use during matches.

Contains only:

- values the drivers actually need
- values that change meaningfully
- values that are cheap or important enough to justify cost

Good competition examples:

- shooter ready / busy
- shooter target preset
- turret on target
- intake state
- color full / not full
- loop rate
- follower pose only if drivers truly use it
- overcurrent/alert booleans

Bad competition examples:

- continuous motor current amps every loop
- raw profile acceleration
- static gear-ratio data
- target values that never change during the match

Cadence:

- refresh at `5 Hz`
- optimize for readability over raw temporal fidelity

## 3. `DEBUG`

Use during tuning and testing.

Contains:

- motor measured velocity
- target velocity
- control error
- applied power
- battery voltage
- profile reference values
- current readings, but throttled

This is where most mechanism tuning telemetry belongs.

Cadence:

- publish every loop to Panels
- allow Driver Station to stay capped at `5-10 Hz`

## 4. `TRACE`

Use only for deep debugging.

Contains:

- nearly everything
- per-controller internal values
- reference states
- rare fault counters
- detailed diagnostics

This should almost never be on in normal driving.

Cadence:

- same as `DEBUG` for now: per-loop on Panels, capped on Driver Station

## The cost model I would use

This is the piece that will make the system actually good.

Every telemetry field should have a **cost class**.

## Cost class A: static

Examples:

- ticks per revolution
- motor hardware name
- configured current alert threshold
- gear ratio
- min/max angle limits

Rule:

- show once at init
- or only on mode entry
- never stream every loop

## Cost class B: cheap runtime state

Examples:

- applied power
- target setpoint
- internal controller error
- current state enum
- boolean flags
- overcurrent alert boolean
- loop dt

Rule:

- safe for regular update
- still do not spam if unchanged

## Cost class C: bulk-cached hardware reads

Examples:

- encoder position
- motor velocity
- current alert / overcurrent state if it comes from the hub's bulk-readable status path

Rule:

- can be used more freely
- still prefer to read once and reuse

## Cost class D: non-bulk hardware reads

Examples:

- continuous motor current in amps
- some advanced device status calls
- values that trigger standalone hardware transactions

Rule:

- throttle
- debug-only by default
- cache for a period like `50-200 ms`

This is where your current-reading note matters most.

## Cost class E: expensive derived/formatting values

Examples:

- long `toString()` objects
- formatted pose strings
- multi-line debug summaries
- string-heavy packet dumps

Rule:

- debug-only
- compute lazily

## What this means in practice for the motor package

The motor package should expose telemetry in three groups.

## 1. Competition-safe fields

These are the fields a motor facade can expose for frequent use:

- target
- measured state
- applied power
- simple `busy`
- simple `atTarget`
- `overCurrent`
- fault booleans

These are useful and mostly cheap.

## 2. Debug fields

These are useful when tuning:

- position error
- velocity error
- reference velocity
- reference acceleration
- battery voltage factor
- dt

These should usually be enabled only in `DEBUG` or `TRACE`.

## 3. Slow diagnostics

These are valuable but expensive:

- continuous motor current
- total hub current draw
- any repeated sensor transaction that is not bulk cached

These should be sampled slower and cached.

## The architecture I recommend

Below is the shape I would implement.

## A. Add telemetry snapshots to the motor package

For the new motor package, add small data classes such as:

- `OpenLoopMotorTelemetrySnapshot`
- `VelocityMotorTelemetrySnapshot`
- `ProfiledMotorTelemetrySnapshot`

The snapshot should not format strings.

It should just expose numbers and booleans.

Example:

```java
public final class VelocityMotorTelemetrySnapshot {
    public final double targetTicksPerSecond;
    public final double measuredTicksPerSecond;
    public final double appliedPower;
    public final boolean overCurrent;
    public final boolean atSpeed;
    public final Double currentAmps;

    public VelocityMotorTelemetrySnapshot(...) { ... }
}
```

`currentAmps` can be nullable or optional if not sampled this cycle.

That is a clean way to represent:

- cheap data available now
- expensive data only available when sampled

## B. Add telemetry providers, not telemetry calls

Each subsystem or motor facade should implement something conceptually like:

```java
public interface TelemetryProvider {
    void collectTelemetry(TelemetryCollector collector, TelemetryMode mode);
}
```

The provider:

- collects from internal state
- gives raw values to a collector
- does not directly call `PanelsTelemetry`

This is much better than direct `telemetryM.addData(...)` scattered everywhere.

## C. Add a central `TelemetryCollector`

This class is the shared aggregator.

Responsibilities:

- accept values from providers
- ignore disabled sections
- suppress static fields after first publish
- suppress unchanged values if desired
- publish to DS / Panels

Conceptually:

```java
collector.add("shooter.target", target, CostClass.CHEAP, TelemetryMode.COMPETITION);
collector.add("shooter.current", currentAmps, CostClass.NON_BULK, TelemetryMode.DEBUG);
collector.add("turret.error", error, CostClass.CHEAP, TelemetryMode.DEBUG);
```

## D. Add a `TelemetryHub`

This is the orchestrator.

Responsibilities:

- store registered providers
- know the active telemetry mode
- know throttling intervals
- publish final telemetry once per cycle
- decide whether the current loop should emit telemetry at all based on mode cadence

Example responsibilities:

- `setMode(TelemetryMode mode)`
- `register(TelemetryProvider provider)`
- `beginLoop(loopTime)`
- `publish(telemetry)`

In the current code, that cadence policy is:

- `OFF` and `COMPETITION`: publish every `0.2 s`
- `DEBUG` and `TRACE`: publish every loop
- always force an immediate publish when entering a new mode

## E. Add throttled samplers for expensive values

I strongly recommend a reusable helper for values like current readings.

Example concept:

```java
public final class ThrottledValue<T> {
    private double lastSampleTime;
    private T cachedValue;
    private final double minIntervalSec;

    public T get(double now, Supplier<T> sampler) { ... }
}
```

Use it for:

- `DcMotorEx.getCurrent(...)`
- hub current draw
- any non-bulk status read

This gives you clean policy:

- current draw at `10 Hz`
- overcurrent boolean every loop if needed

## Why your "methods for debug telemetry and competition telemetry" idea is good

The idea is good, but I would refine it slightly.

Instead of this:

```java
motor.addDebugTelemetry(...)
motor.addCompetitionTelemetry(...)
```

I would prefer this:

```java
motor.collectTelemetry(collector, TelemetryMode.DEBUG);
motor.collectTelemetry(collector, TelemetryMode.COMPETITION);
```

Reason:

- one API
- cleaner extension to `TRACE` and `OFF`
- the caller chooses mode
- easier to test

So the underlying idea is right:

- a motor should know which of its fields are useful in competition
- a motor should know which fields are useful for debugging

But that knowledge should be expressed through a provider/collector contract, not direct telemetry side effects.

## What should live in the motor package vs subsystem layer

This boundary matters a lot.

## Motor package should own

- motor telemetry snapshots
- controller/debug values that are motor-native
- cost classification hints
- optional telemetry-provider implementations for motor facades

Examples:

- measured velocity
- target velocity
- applied power
- at-speed
- overcurrent
- controller error
- reference state

## Subsystems should own

- mechanism semantics
- presets
- game-state interpretation
- derived driver-facing booleans

Examples:

- `shooter ready`
- `turret on target`
- `intake full`
- `far shot selected`
- `hang mode active`

So:

- motor package exposes clean motor facts
- subsystems transform those into robot meaning

That is the right separation.

## How static telemetry should be handled

You specifically mentioned useless static telemetry.

I would ban static values from the normal loop.

Static values should be emitted only in one of these cases:

1. `init()`
2. mode switch
3. debug command
4. explicit "show config" page

Examples:

- motor type
- ticks per rev
- max power
- configured PID values
- angle limits

Do **not** publish them continuously in the control loop.

Your legacy `MotorConfig` is a good example of what to avoid here.

## How unchanged runtime telemetry should be handled

Another important improvement: suppress values that have not meaningfully changed.

Examples:

- intake state does not need to be re-added every loop if unchanged
- preset name does not need constant re-publication
- target velocity does not need to be resent unless it changed or the mode requires it

I would support two suppression policies:

## Policy 1: publish-on-change

Best for:

- enums
- booleans
- named presets
- alerts

## Policy 2: publish-on-interval

Best for:

- measured velocities
- error values
- loop-rate
- pose

This gives telemetry the feeling of being alive without wasting bandwidth.

## Suggested telemetry sections

A good system is not only about cost. It is also about organization.

I would group telemetry into sections.

## Section: `system`

Competition-safe:

- loop Hz
- loop dt
- battery voltage
- RC mode

Debug:

- average loop time
- worst loop time
- optional segment timings

## Section: `drive`

Competition:

- heading
- driver mode
- maybe pose summary

Debug:

- pose x/y/heading
- follower velocity
- follower error

## Section: `shooter`

Competition:

- target preset
- running
- ready / busy
- impact detected
- overcurrent alert

Debug:

- target velocity
- measured velocity
- filtered velocity
- applied power
- control error
- current amps, throttled

## Section: `turret`

Competition:

- on target
- limelight mode / field-aim mode
- overcurrent alert

Debug:

- target angle
- measured angle
- applied power
- position error
- reference velocity
- reference acceleration
- current amps, throttled

## Section: `intake`

Competition:

- state
- full / not full
- alert if jam / overcurrent

Debug:

- power
- current amps, throttled
- sensor distances if explicitly enabled

## Section: `vision`

Competition:

- limelight valid / not valid
- maybe tx bucket like `left/center/right`

Debug:

- raw `tx`
- pipeline
- latency if available

## Concrete implementation plan

I would implement this in phases.

## Phase 1: stop the bleeding

Goal:

- reduce telemetry cost without a huge refactor

Steps:

1. remove direct telemetry from the new motor controllers and keep it out
2. stop adding static values in loop methods
3. throttle all current-amp reads
4. introduce explicit `COMPETITION` vs `DEBUG` selection in `MainTeleOp`

Even this phase alone would already improve clarity.

## Phase 2: create the shared telemetry infrastructure

Add:

- `TelemetryMode`
- `TelemetryCostClass`
- `TelemetryProvider`
- `TelemetryCollector`
- `TelemetryHub`

This is the core architecture layer.

## Phase 3: migrate the motor package first

Add provider/snapshot support to:

- `MetaMotor`
- `OpenLoopMotor`
- `VelocityControlledMotor`
- `ProfiledPositionMotor`

Do not start with all subsystems first.

Why:

- the motor package is the lowest shared layer
- many subsystem telemetry needs come from motor state
- once the motor package is clean, subsystem telemetry becomes easier

## Phase 4: migrate subsystems

Start with:

1. `Shooter`
2. `Turret`
3. `Intake`

Then move `MainTeleOp` to consume the shared architecture rather than manually building telemetry.

## Phase 5: add advanced features

Optional future improvements:

- DS telemetry profiles toggled by gamepad
- telemetry page cycling
- one-shot config dump
- trace logging to logcat
- timing instrumentation for loop segments
- separate per-sink collector paths if Panels and Driver Station ever need different field sets in the same mode

## Recommended class design

Here is the shape I would actually aim for.

## `TelemetryMode`

```java
public enum TelemetryMode {
    OFF,
    COMPETITION,
    DEBUG,
    TRACE
}
```

## `TelemetryCostClass`

```java
public enum TelemetryCostClass {
    STATIC,
    CHEAP,
    BULK_CACHED,
    NON_BULK,
    FORMATTED
}
```

## `TelemetryField`

This can store:

- key
- value supplier
- mode visibility
- cost class
- publish policy
- section

The supplier lets you compute only when needed.

That is very important for expensive values.

## `TelemetryCollector`

Responsibilities:

- gather fields from providers
- decide if a field should publish now
- optionally compare to last value
- store output in section order

## `TelemetryProvider`

```java
public interface TelemetryProvider {
    void collectTelemetry(TelemetryCollector collector, TelemetryMode mode);
}
```

## `MotorTelemetryProvider`

You can either:

- let each motor facade implement `TelemetryProvider`
- or add separate adapter classes

I would lean toward separate adapters if you want stricter separation, for example:

- `VelocityMotorTelemetryProvider`
- `ProfiledMotorTelemetryProvider`

But if you want less code, having the facade implement the provider is acceptable as long as it still does not know about `PanelsTelemetry`.

## Should the motor package own the telemetry architecture?

Mostly yes, but not completely.

Here is the best split.

## Motor package should own the telemetry **contract**

It should define:

- what motor-native telemetry exists
- what snapshots look like
- how providers expose them

## App / subsystem layer should own the telemetry **selection**

It should decide:

- what mode is active
- which sections are enabled
- what to display to drivers
- what is worth showing during testing

This avoids making the motor package too opinionated about match strategy or driver UX.

## What to do with `PanelsTelemetry`

Keep it at the edge.

That means:

- one central telemetry output adapter uses `PanelsTelemetry`
- providers and collectors do not

This is the exact same design principle your motor rewrite docs already prefer.

## Recommended first implementation targets in your codebase

If I were implementing this in your repo, I would start with these targets:

1. `MainTeleOp`
2. `Shooter`
3. `Turret`
4. `VelocityControlledMotor`
5. `ProfiledPositionMotor`
6. `MetaMotor`

And I would explicitly avoid adding any new telemetry logic into:

- `PIDFFVelocityController`
- `PIDFFPositionController`
- `TrapezoidalMotionProfileController`

because those should stay pure.

## A very practical starting checklist

If you want a low-risk path, do this:

1. Create `TelemetryMode`
2. Create `TelemetryProvider`
3. Create `TelemetryCollector`
4. Create a `ShooterTelemetrySnapshot`
5. Make `Shooter` implement `collectTelemetry(...)`
6. Add `TelemetryHub` to `MainTeleOp`
7. Switch `MainTeleOp` from manual telemetry lines to hub publish
8. Then repeat for `Turret`

This proves the architecture before you generalize it everywhere.

## Final recommendation

Your idea is good, but I would sharpen it into this rule:

> The motor package should define telemetry state and telemetry contracts, while the OpMode chooses telemetry mode and the output adapter chooses how to render it.

That gives you:

- reusable telemetry across subsystems
- clear competition vs debug separation
- less repeated static spam
- less accidental expensive polling
- a future-proof architecture that fits the motor rewrite direction

The two most important decisions are:

1. separate telemetry collection from telemetry rendering
2. treat bulk-cached booleans and non-bulk continuous readings as different classes of data

That second point is especially important given your note about current alerts vs current readings.

## Summary of the architecture in one sentence

Build a telemetry hub where motor facades and subsystems provide structured snapshots, each field is tagged by mode and cost, expensive values are throttled, static values are one-shot, and only the final adapter talks to `PanelsTelemetry`.
