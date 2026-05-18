# Telemetry Package Structure

This package contains the shared telemetry infrastructure for the `pedroPathing.main` robot code. Its job is to keep telemetry collection structured and cheap enough for FTC control loops, while still allowing richer debugging when needed.

At a high level, the package separates telemetry into four concerns:

1. mode selection
2. provider registration
3. field collection and publish policy
4. snapshot/throttling utilities for expensive reads

That separation matters because the robot has both very cheap values, like booleans or cached state, and more expensive values, like current readings. The package gives the OpMode and subsystems one place to manage that tradeoff instead of scattering `addData()` calls everywhere.

## Package Contents

The package currently contains these files:

- `TelemetryHub.java`
- `TelemetryCollector.java`
- `TelemetryProvider.java`
- `TelemetryMode.java`
- `TelemetryCostClass.java`
- `TelemetryPublishPolicy.java`
- `ThrottledValue.java`
- `ShooterTelemetrySnapshot.java`
- `TurretTelemetrySnapshot.java`

They are not all at the same architectural level. Some are orchestration classes, some are contracts, and some are simple data holders.

## Architecture Overview

The telemetry flow is:

`OpMode/subsystem state -> TelemetryProvider -> TelemetryCollector -> TelemetryHub publish -> Driver Station + Panels`

In the current codebase this flow is driven primarily by [`MainTeleOp.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java), which:

- owns a `TelemetryHub`
- registers itself as a provider
- registers subsystem providers like `Shooter` and `Turret`
- chooses the active `TelemetryMode`
- calls `telemetryHub.publish(...)` once per loop

This gives the codebase a single publish point instead of multiple classes pushing directly to telemetry outputs.

## Output Cadence

The package now uses mode-aware publish cadence instead of treating every mode the same.

- `OFF` and `COMPETITION` publish through the hub at `5 Hz` (`0.2 s` interval).
- `DEBUG` and `TRACE` publish through the hub every loop.
- Driver Station telemetry is still capped separately at `10 Hz` by the FTC telemetry transmission interval, even when the hub is publishing every loop.

This means:

- competition/off telemetry is intentionally slower on both sinks
- debug/trace telemetry stays high-resolution on Panels
- Driver Station telemetry remains human-readable and bandwidth-limited regardless of mode

## Core Orchestrator

### `TelemetryHub`

[`TelemetryHub.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryHub.java) is the top-level coordinator.

Responsibilities:

- stores the list of registered `TelemetryProvider`s
- stores persistent field publication state across loops
- stores the active telemetry mode
- constructs a fresh `TelemetryCollector` for each publish cycle
- invokes all providers in registration order
- forwards the final field set to both telemetry sinks
- enforces mode-dependent output cadence before building a collector

Important details:

- `providers` is a `List`, so output order is stable relative to registration order.
- `fieldStates` is retained across loops. That is what allows publish policies such as "only on change" or "only when entering a mode" to work over time.
- `lastPublishedMode` is tracked so the collector can tell whether the current cycle is the first cycle after a mode switch.
- `lastOutputTimeSeconds` is tracked so `OFF` and `COMPETITION` can be rate-limited to `5 Hz` while `DEBUG` and `TRACE` still publish every loop.

`TelemetryHub` does not know how a subsystem computes values. It only knows how to ask for values and publish the finished set.

## Collection Layer

### `TelemetryCollector`

[`TelemetryCollector.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryCollector.java) is the per-loop aggregator used by providers.

Responsibilities:

- accepts telemetry fields from providers
- filters fields by minimum telemetry mode
- applies publish policies
- remembers pending fields for this loop only
- publishes those fields to `TelemetryManager` and FTC `Telemetry`

The collector is intentionally short-lived. A new instance is created every time `TelemetryHub.publish(...)` runs. What persists between loops is not the collector itself, but the shared `fieldStates` map passed into it by the hub.

### Field naming

Each field is identified internally as:

`section + "." + key`

Examples from current usage:

- `system.loop_hz`
- `vision.locked`
- `shooter.current_amps`
- `turret.reference_velocity_tps`

This gives the package a flat output model with namespaced keys instead of a nested object tree.

### `FieldState`

`TelemetryCollector.FieldState` is a private per-field memory record used to decide whether a value should be republished.

It stores:

- `lastValue`
- `hasPublished`
- `lastMode`
- `costClass`

In the current implementation, `costClass` is recorded but not actively used to throttle or suppress publication. It is metadata that documents how expensive a field is and leaves room for future policy decisions.

### Publish semantics

When a provider calls `collector.add(...)`, the collector:

1. ignores `null` values
2. ignores values whose minimum mode is above the current mode
3. looks up persistent field state by `section.key`
4. checks whether the field should publish under its policy
5. updates the stored field state
6. queues the field in `pendingFields`

Only queued fields are sent to outputs at the end of a publish cycle. In `DEBUG` and `TRACE`, that is normally every loop. In `OFF` and `COMPETITION`, the hub only opens a publish cycle every `0.2` seconds unless the mode has just changed.

## Provider Contract

### `TelemetryProvider`

[`TelemetryProvider.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryProvider.java) is the interface that subsystems and OpModes implement when they want to contribute telemetry.

```java
void collectTelemetry(TelemetryCollector collector, TelemetryMode mode);
```

This is the main decoupling point in the package.

A provider:

- knows its own robot state
- decides which values to expose
- tags those values with mode and cost
- does not publish directly to Driver Station or Panels telemetry

Current providers outside this package include:

- [`MainTeleOp.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java)
- [`Shooter.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java)
- [`Turret.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java)

This is an important design choice: the telemetry package owns the contract, but the surrounding robot code owns the actual domain values.

## Visibility Model

### `TelemetryMode`

[`TelemetryMode.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryMode.java) defines the four visibility levels:

- `OFF`
- `COMPETITION`
- `DEBUG`
- `TRACE`

The enum is ordered from least verbose to most verbose. The method `includes(...)` uses ordinal comparison, which means:

- `TRACE` includes everything
- `DEBUG` includes `DEBUG`, `COMPETITION`, and `OFF`
- `COMPETITION` includes `COMPETITION` and `OFF`
- `OFF` includes only `OFF`

`next()` rotates through the modes cyclically. In [`MainTeleOp.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java), that is bound to `gamepad2.back`, which lets operators move between telemetry profiles at runtime.

Mode currently affects both content and cadence:

- `OFF`: minimal content, `5 Hz` hub publish rate
- `COMPETITION`: driver-facing content, `5 Hz` hub publish rate
- `DEBUG`: expanded tuning content, per-loop hub publish rate
- `TRACE`: deepest diagnostics, per-loop hub publish rate

## Cost Metadata

### `TelemetryCostClass`

[`TelemetryCostClass.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryCostClass.java) classifies the expected cost of producing a value:

- `STATIC`
- `CHEAP`
- `BULK_CACHED`
- `NON_BULK`
- `FORMATTED`

The code uses these labels to describe intent:

- `STATIC` means configuration-like data that rarely changes.
- `CHEAP` means inexpensive state or derived values.
- `BULK_CACHED` means data that is expected to come from efficient bulk hardware reads or similarly cheap access paths.
- `NON_BULK` means potentially expensive hardware transactions such as current measurement.
- `FORMATTED` means values whose main cost is formatting or object-to-string conversion.

At the moment this enum is descriptive rather than prescriptive. The package does not yet auto-throttle based on cost class alone. Instead, providers use cost class together with `TelemetryMode`, `TelemetryPublishPolicy`, and `ThrottledValue` to express how a field should behave.

## Publish Policies

### `TelemetryPublishPolicy`

[`TelemetryPublishPolicy.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryPublishPolicy.java) defines when a field should be emitted:

- `ALWAYS`
- `ON_CHANGE`
- `ON_MODE_ENTRY`

How they behave:

- `ALWAYS`: publish every loop in which the field is added and the mode allows it.
- `ON_CHANGE`: publish only the first time, or when `value.equals(lastValue)` becomes false.
- `ON_MODE_ENTRY`: publish the first time, or when telemetry mode changes, or when the field was last published under a different mode.

The strongest current example is in [`Turret.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java), where static PID and motion-profile constants are published in `TRACE` mode using `ON_MODE_ENTRY`. That prevents constant re-sending of configuration values during normal loops while still making them visible when deep diagnostics are enabled.

## Throttling Utility

### `ThrottledValue<T>`

[`ThrottledValue.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/ThrottledValue.java) is a small caching helper for values that are too expensive or unnecessary to resample every loop.

It stores:

- a minimum sample interval
- the last sample time
- the cached value

Its `get(nowSeconds, sampler)` method only calls the supplier when either:

- no cached value exists yet
- enough time has elapsed since the last sample

Otherwise it reuses the cached value.

This is how the current system handles reads like motor current:

- `Shooter` throttles `getCurrentAmps()` to 0.1 seconds
- `Turret` throttles `getCurrentAmps()` to 0.1 seconds
- `MainTeleOp` throttles intake current and total robot current

That keeps debug telemetry available without forcing high-cost hardware calls every loop.

## Snapshot Objects

The package also contains small immutable snapshot classes:

- [`ShooterTelemetrySnapshot.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/ShooterTelemetrySnapshot.java)
- [`TurretTelemetrySnapshot.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TurretTelemetrySnapshot.java)

These classes are plain data containers. They do not publish telemetry and they do not contain business logic.

Their purpose is to:

- gather mechanism state into one immutable object
- keep telemetry code from repeatedly poking many subsystem getters
- separate "compute state" from "choose what to publish"

### `ShooterTelemetrySnapshot`

This snapshot packages shooter-related state such as:

- target velocity
- measured velocity
- filtered velocity
- applied power
- hood angle
- running/busy flags
- impact detection
- over-current state
- optional current draw

It is created by [`Shooter.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java) in `getTelemetrySnapshot(...)`, then consumed by `collectTelemetry(...)`.

### `TurretTelemetrySnapshot`

This snapshot packages turret-related state such as:

- control mode
- goal-relative and target angles
- measured angle and velocity
- applied power
- reference motion-profile velocity and acceleration
- over-current and limit flags
- optional current draw

It is created by [`Turret.java`](/D:/projects/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java) in `getTelemetrySnapshot(...)`, then consumed by `collectTelemetry(...)`.

The important pattern is that subsystem code can build a snapshot once, then use that snapshot to feed multiple collector fields.

## How This Package Is Used Today

The current integration pattern looks like this:

### In `MainTeleOp`

- construct `TelemetryHub`
- clear and register providers during `init()`
- set the default mode to `COMPETITION`
- optionally cycle modes during `loop()`
- call `telemetryHub.publish(...)` once per loop, while letting the hub decide whether this loop should actually emit telemetry

### In providers like `Shooter` and `Turret`

- implement `TelemetryProvider`
- optionally build a snapshot
- optionally use `ThrottledValue` for expensive measurements
- call `collector.add(...)` for each field

This gives the codebase a single telemetry pipeline while still letting each subsystem own its own telemetry vocabulary.

## Design Strengths

The package is small, but it already establishes several good boundaries:

- telemetry output is centralized
- providers do not talk directly to `TelemetryManager`
- mode-based visibility is explicit
- mode-based publish cadence is explicit
- expensive reads can be throttled
- static/config values can avoid loop spam
- subsystem telemetry can be composed through snapshots

Those boundaries are especially useful in FTC code because they reduce accidental loop-cost growth when debugging features get added over time.

## Current Limitations

The package also has a few limits worth knowing if you extend it:

- `TelemetryCostClass` is stored as metadata but not yet used for automatic scheduling or suppression.
- `TelemetryCollector.publish(...)` still pushes every queued field to both outputs once a publish cycle begins; per-sink cadence differences are currently handled by the hub plus the Driver Station transmission interval rather than by separate collector paths.
- field ordering is provider-driven and flat, not grouped into rendered sections beyond the `section.key` naming convention.
- `ON_CHANGE` relies on `equals(...)`, so custom value types should have meaningful equality if they are ever used with that policy.
- there is no built-in numeric deadband, rate limiting, or sampling policy beyond what providers manually implement with `ThrottledValue`.

None of those are necessarily problems, but they define the current scope of the package.

## Practical Extension Guidance

If you add a new subsystem and want it to participate cleanly in this telemetry architecture:

1. implement `TelemetryProvider`
2. create a snapshot class if the subsystem has a moderate amount of telemetry state
3. use `TelemetryMode` to separate competition-safe fields from debug-only fields
4. tag each field with a reasonable `TelemetryCostClass`
5. use `TelemetryPublishPolicy.ON_MODE_ENTRY` or `ON_CHANGE` for static or rarely changing values
6. use `ThrottledValue` for non-bulk or hardware-expensive reads
7. register the provider with `TelemetryHub` from the owning OpMode

That keeps new telemetry consistent with the rest of the package.

## Summary

This telemetry package is a lightweight framework for structured robot telemetry. `TelemetryHub` manages providers and publication, `TelemetryCollector` enforces per-field rules for a single loop, `TelemetryProvider` gives subsystems a common contract, `TelemetryMode` and `TelemetryPublishPolicy` control visibility and repetition, `TelemetryCostClass` documents value cost, and `ThrottledValue` prevents expensive diagnostics from overwhelming the loop. The snapshot classes round out the package by giving complex subsystems a clean way to package state before publishing it.

In the current implementation, `TelemetryHub` also controls refresh cadence: `OFF` and `COMPETITION` emit at `5 Hz`, `DEBUG` and `TRACE` emit every loop, and Driver Station traffic is capped separately at `10 Hz`.
