# Turret SOTM Implementation Advisor

## Purpose

This document is meant to be the implementation advisor for adding SOTM support to the turret.

It should help you:

- understand the control theory behind the change
- understand what is already correct in the current system
- identify the real problem
- implement the feature in the right order
- verify the feature without accidentally blaming the controller for an upstream targeting issue

This is not only a design note. It is the step-by-step guide to follow while coding.

## Executive Summary

The turret controller itself is already fundamentally correct.

Your existing system already contains the important P-P-F pieces:

- position correction through `kp * positionError`
- velocity correction through the current `kd * velocityError`
- feedforward through `ks`, `kv`, and `ka`

The feature you need to implement is **not a new controller**.

The real work is:

1. replace the current moving-shot lead model with a physics-based angular lead
2. compute lead from tangential chassis motion only
3. source flight time from the same shooter model already used by the robot
4. teach the turret profile to track a nonzero target velocity during moving shots

If you keep that distinction clear, the implementation stays focused and much less risky.

## The Theory You Must Preserve

### P-P-F is already present

Your controller already behaves like a P-P-F architecture:

| Concept | Current Behavior |
|---|---|
| Position loop | Drives turret toward reference angle |
| Velocity loop | `kd` acts on reference velocity minus measured velocity |
| Feedforward | `ks + kv * vRef + ka * aRef` |

This means:

- do not start by rewriting PID
- do not assume shot errors mean the power law is bad
- do not retune everything before fixing target generation

### Why SOTM needs nonzero turret velocity

During a stationary shot, it is correct for the turret profile to settle to:

- target angle
- zero velocity

During a moving shot, that is no longer correct.

If the robot translates across the field, the goal angle in robot-local coordinates drifts continuously even when the chassis is not rotating. The turret must rotate continuously just to stay on target.

So for SOTM:

- target angle alone is not enough
- target terminal velocity should not be zero

The compensation velocity is:

\[
\omega_{compensation} = \frac{v_{tangential}}{d}
\]

where:

- `vTangential` is robot velocity perpendicular to the goal line
- `d` is distance to the goal

That velocity must be fed continuously into the turret reference path while tracking a moving shot.

### Why the current lead model is wrong

The current code concept is:

```java
compensatedPose = pose + leadFactor * velocity * distance
```

This is wrong for two reasons:

1. it uses the full velocity vector instead of just the tangential component
2. it uses a magic constant instead of projectile flight time

Only tangential velocity changes aim angle directly.

Radial velocity:

- changes distance
- may influence flight time indirectly
- does not directly create bearing correction

That is why the current approach can under-correct sideways motion and over-correct forward or backward motion.

## The Problem Statement

You are not trying to make the turret "aim better" in a vague sense.

You are specifically trying to solve this:

- when the robot is moving, the turret should aim at the place the goal will appear relative to the projectile's flight time
- while doing so, the turret should track the angular drift caused by chassis translation instead of trying to come to rest

That breaks into two concrete missing capabilities:

1. **physics-based moving-shot lead angle**
2. **nonzero moving-shot reference velocity**

Both must be implemented. Fixing only one will still leave visible error.

## The Solution Model

The correct SOTM pipeline is:

1. compute vector from robot to goal
2. compute distance to goal
3. decompose robot velocity into radial and tangential components
4. get projectile flight time from the shooter lookup/simulation
5. compute lead angle from tangential motion during flight
6. compute compensation angular velocity from tangential motion over distance
7. build final turret target angle as base angle plus lead
8. feed both target angle and target velocity into the turret control path

In short:

- lead is an **angular offset**
- moving-shot tracking is a **velocity bias**

## Implementation Order

Follow this order exactly. Do not skip ahead.

### Phase 1: Prepare and map the code paths

Before changing code, identify the exact locations involved.

You need to confirm:

- where `lookToGoal()` builds the stationary target angle
- where `lookToGoalWhileMoving()` currently applies pose translation
- where shooter lookup or shot simulation data is stored
- where the profile/controller currently assumes zero target velocity
- what telemetry already exists for turret reference angle and velocity

Output of this phase:

- a list of files and methods to touch
- a list of files and methods that should remain unchanged

### Phase 2: Build the SOTM calculation layer first

Do not start by editing the controller.

First add a dedicated moving-shot calculation path that returns all physically meaningful outputs.

This path should calculate:

- `distance`
- `goalDir`
- `tangentDir`
- `vTangential`
- `vRadial`
- `flightTime`
- `leadAngle`
- `baseAngle`
- `targetAngle`
- `omegaCompensation`

Recommended shape:

- either a small helper method in the turret subsystem
- or a small dedicated helper class/data object if you want better separation and telemetry visibility

Acceptance criteria for this phase:

- you can log all quantities above without changing the controller
- you can compare current target angle vs new target angle in telemetry

### Phase 3: Replace pose-shift lead with angular lead

Once the SOTM calculation layer exists, replace the current moving-shot correction strategy.

Old idea:

```java
compensatedPose = pose + leadFactor * velocity * distance
```

New idea:

```java
Vector2d toGoal     = goal.minus(robotPosition);
double distance     = toGoal.norm();
Vector2d goalDir    = toGoal.div(distance);
Vector2d tangentDir = new Vector2d(-goalDir.y, goalDir.x);

double vTangential  = robotVelocity.dot(tangentDir);
double vRadial      = robotVelocity.dot(goalDir);
double flightTime   = shooterLookup.getFlightTime(distance);
double leadAngle    = Math.atan2(vTangential * flightTime, distance);

double baseAngle    = Math.atan2(toGoal.y, toGoal.x) - robotHeading;
double targetAngle  = baseAngle + leadAngle;
```

Implementation rules:

- do not translate pose anymore for SOTM
- compute lead in angular space
- apply lead after computing base angle in the turret-local frame

Acceptance criteria for this phase:

- zero robot velocity produces near-zero lead angle
- pure radial motion produces much smaller lead than pure tangential motion
- lead sign flips correctly when tangential velocity flips

### Phase 4: Connect to real flight-time data

This phase is critical.

Do not leave a fake placeholder like:

- constant flight time
- `distance / projectileSpeed`
- old `leadFactor`

if your robot already has shooter simulation or lookup data that is more accurate.

Required work:

- find the existing shooter lookup/simulation entry point
- expose `getFlightTime(distance)` or equivalent
- ensure the same distance convention is used by both shooter selection and turret lead

Why this matters:

- if shooter speed uses one physics model and turret lead uses another, you reintroduce mismatch even if the formulas look correct

Acceptance criteria for this phase:

- flight time can be printed and inspected for short, medium, and long shots
- flight time changes sensibly with distance
- no separate hidden "lead tuning constant" remains in the moving-shot path

### Phase 5: Add moving-shot reference velocity

After angle lead is correct, add the second missing piece:

\[
\omega_{compensation} = \frac{v_{tangential}}{d}
\]

This is not optional if you want the profile to stop lagging behind a moving target.

Required work:

- compute `omegaCompensation` every loop
- feed it as the target/reference velocity for the turret while SOTM is active

Important behavior rule:

- stationary shot: target velocity = `0`
- moving shot: target velocity = `omegaCompensation`

Acceptance criteria for this phase:

- reference turret velocity is no longer forced to zero during moving-shot tracking
- reference velocity sign matches the sign of tangential motion
- measured velocity can settle near the nonzero reference during steady sideways motion

### Phase 6: Extend the profile interface only as much as needed

At this point, you may need to extend the current turret control path so a target velocity can be provided.

Do the smallest safe extension.

The implementation goal is:

- preserve current stationary behavior
- preserve current angle limits
- preserve current feedforward/PID logic
- only add the ability to specify nonzero target velocity when desired

Possible implementation direction:

- keep the existing target-position call for stationary aiming
- add a second call or target-state path for moving-shot aiming
- internally build a target state containing position and velocity

Do not let this phase turn into a controller rewrite.

Acceptance criteria for this phase:

- old stationary path still works unchanged
- new path can provide both target angle and target velocity
- switching between the two is explicit and testable

### Phase 7: Add guards and failure handling

Before field testing, add guards for the known bad cases.

Required guards:

- minimum allowed distance to avoid divide-by-zero
- handling for missing or invalid shooter lookup data
- fallback to zero lead and zero compensation velocity if inputs are invalid
- optional filtering if robot velocity is noisy

Recommended telemetry flags:

- `sotm valid`
- `distance valid`
- `flight time valid`
- `velocity valid`

Acceptance criteria for this phase:

- invalid inputs fail safe instead of producing huge turret commands
- telemetry makes it obvious why SOTM was disabled or degraded

## What To Change

This section is the practical coding checklist.

### A. Turret moving-shot target generation

Change:

- the moving-shot aiming method

Implement:

- velocity decomposition
- angular lead calculation
- compensation velocity calculation
- direct target-angle output

Keep:

- stationary `lookToGoal()` logic as the fallback reference
- existing turret angle clipping behavior

### B. Shooter lookup interface

Change:

- expose flight time from existing shot data

Implement:

- `getFlightTime(distance)` or equivalent

Keep:

- existing shot selection physics model

### C. Profile target representation

Change:

- target representation must support target velocity during SOTM

Implement:

- a path for target angle plus target velocity

Keep:

- existing PIDF structure
- existing feedforward law
- existing battery scaling

### D. Telemetry and testing hooks

Change:

- add SOTM-specific visibility

Implement telemetry for:

- base angle
- lead angle
- target angle
- distance
- vTangential
- vRadial
- flight time
- omegaCompensation
- reference velocity
- measured velocity

## What Not To Change First

Avoid these mistakes during implementation:

- do not retune PID before the new target model exists
- do not replace the profiled controller unless testing proves it is necessary
- do not collapse stationary and moving-shot paths into one opaque method too early
- do not hide the new calculations from telemetry
- do not keep the old `leadFactor` logic alive in parallel unless it is clearly marked as legacy

## Suggested Data Model

The implementation gets easier if the SOTM calculation returns a structured result.

Suggested fields:

```java
class SotmTarget {
    double distance;
    double baseAngleRadians;
    double leadAngleRadians;
    double targetAngleRadians;
    double flightTimeSeconds;
    double vTangential;
    double vRadial;
    double omegaCompensation;
    boolean valid;
}
```

This is not mandatory, but it makes debugging and verification much easier.

## Verification Plan

### Telemetry verification

Before firing a shot, verify on telemetry:

- `leadAngle` is near zero when robot velocity is near zero
- `omegaCompensation` is near zero when robot velocity is near zero
- tangential motion changes lead more than radial motion
- target angle changes smoothly
- reference velocity is nonzero only when SOTM is active

### Driving tests

Run these in order.

1. Stationary test

- confirm no regression in normal aiming
- confirm SOTM values collapse to near-zero correction when stationary

2. Sideways-only motion

- confirm lead angle sign is correct
- confirm target velocity sign is correct
- confirm the turret no longer lags as if it is trying to stop on target

3. Toward-goal motion

- confirm radial motion does not create large angular lead

4. Away-from-goal motion

- confirm similar behavior with reversed radial direction

5. Diagonal motion

- confirm correction follows tangential component, not total speed

6. Multiple distances

- confirm lead changes with flight time and does not behave like a constant-factor hack

### Shot-result verification

When firing, watch for these patterns:

- misses in the direction of motion during sideways driving suggest under-lead or insufficient compensation velocity
- misses opposite the direction of motion suggest over-lead or sign error
- distance-dependent inconsistency suggests flight-time mismatch
- oscillation around the goal while moving suggests the reference velocity path is wrong or noisy

## Risk Checklist

Check these while implementing:

- sign convention of `leadAngle`
- sign convention of `omegaCompensation`
- same units everywhere
- same distance convention as shooter lookup
- same coordinate frame assumptions as `lookToGoal()`
- safe behavior when `distance` is too small
- safe behavior when lookup data is unavailable

## Minimum Acceptable Implementation

If you need a staged rollout, the minimum acceptable version is:

1. replace pose-shift lead with tangential-velocity-based angular lead
2. use real flight time from shooter data
3. add telemetry for the new SOTM quantities

This will already remove the biggest conceptual error.

## Full Implementation

The full feature is not complete until all of these are true:

- moving-shot lead uses tangential velocity only
- lead uses shooter-based flight time
- lead is applied as an angular offset
- turret profile tracks a nonzero reference velocity during motion
- stationary fallback still works
- telemetry exposes the full chain
- field tests pass across multiple distances and motion directions

## Final Advice While Coding

Use this order while working:

1. instrument the system
2. replace the lead model
3. connect real flight time
4. add compensation velocity
5. test stationary behavior again
6. test sideways motion
7. test radial motion
8. only then consider tuning

If something looks wrong, debug in this order:

1. target geometry
2. tangential/radial decomposition
3. flight time source
4. lead angle sign and magnitude
5. compensation velocity sign and magnitude
6. only after that, controller behavior

That order matters. Most failures here will come from target generation, not from the PIDF core.
