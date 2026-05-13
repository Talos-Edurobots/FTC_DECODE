# FTC Loop Optimization Report

## Goal

Increase the control-loop rate of this robot code so PID controllers, turret behavior, shooter response, and TeleOp feel more responsive.

This report focuses on:

- `MainTeleOp` and the competition TeleOp path
- the custom motor package under `pedroPathing/main/motor`
- hardware I/O patterns that affect FTC loop speed

The current project is using FTC SDK `11.1.0` according to `build.dependencies.gradle`.

The recommendations below are ranked by likely payoff.

## Short version

The biggest wins in this codebase are not Java micro-optimizations first. They are:

1. stop doing duplicate high-cost subsystem work in the same loop
2. stop sending identical motor and servo commands every loop
3. stop formatting and transmitting so much telemetry every cycle
4. centralize sensor snapshots like battery voltage and other slow-changing values
5. reduce object churn inside the motor controllers

You are already doing one important thing correctly: `HardwareManager` enables `LynxModule.BulkCachingMode.MANUAL` and clears the cache once per loop, which is the FTC SDK's fastest bulk-read strategy when used carefully.

## What the code already does well

### 1. Manual bulk caching is already enabled

`HardwareManager` sets every hub to `MANUAL` caching and clears the bulk cache once per loop:

- `HardwareManager.java:17-25`

That is the correct high-performance FTC pattern. The SDK sample `ConceptMotorBulkRead` explicitly says that manual bulk caching gives the shortest cycle times when you clear once per control cycle.

### 2. The color sensors are already throttled

`ColorSensors` only refreshes at `5 Hz` by default instead of every loop:

- `ColorSensors.java:10-18`
- `ColorSensors.java:35-40`

That is a good example of a subsystem that already separates control-loop frequency from sensor-update frequency.

## Highest-priority optimizations

## 1. Remove the second `follower.update()` from `MainTeleOp.loop()`

### Where it happens

- first call: `MainTeleOp.java:140-141`
- second call: `MainTeleOp.java:237-240`

### Why this matters

This is the single clearest hotspot in the TeleOp loop.

At the start of the loop you do:

1. `hardwareManager.update()` which clears manual bulk cache
2. `follower.update()`

Later in the same loop you do another `follower.update()` without clearing bulk cache again.

With manual bulk caching, repeated encoder reads in the same cycle are served from the same cached bulk data. That means the second `follower.update()` is very likely:

- using the same sensor snapshot
- recomputing localization / follower logic again
- potentially issuing more motor writes again
- burning CPU without gaining fresher data

So this is not only a possible CPU waste. It may also be conceptually redundant because the second pass does not necessarily have newer hub data.

### Recommendation

Refactor the loop so follower work happens once per cycle:

- clear cache once
- read/update pose once
- compute teleop command
- send drive output once

If the Pedro follower API requires separate "state update" and "command update" phases, wrap that in your own layer so sensor reads still happen once.

### Expected impact

High. This can improve both:

- loops per second
- consistency of loop timing

It is also the cleanest place to reduce duplicated drivetrain work.

## 2. Centralize battery-voltage reads instead of scanning sensors in each subsystem

### Where it happens

- `Shooter.java:73-80`
- `Shooter.java:147-155`
- `Turret.java:190-194`
- `Turret.java:268-276`

### Why this matters

Both shooter and turret independently iterate through `hwmap.voltageSensor` every update. Battery voltage changes slowly relative to a 50-200 Hz robot loop. Reading it per subsystem per loop is unnecessary.

Even if each read is not your single biggest cost, duplicated hardware access adds up. More importantly, it makes the loop less deterministic.

### Recommendation

Read battery voltage once in a top-level loop context and pass it down:

- once per loop, or
- once every `50-100 ms` with a cached value

For example, `HardwareManager` could own:

- cached battery voltage
- timestamp of last refresh
- `beginLoop()` that clears bulk cache and optionally refreshes slow-changing values

Then `Shooter` and `Turret` consume the same loop snapshot instead of re-querying hardware.

### Expected impact

Medium to high. The CPU gain is modest, but it removes repeated hardware work from two hot subsystems and makes loop timing cleaner.

## 3. Suppress identical motor power writes

### Where it happens

- `MetaMotor.java:70-73`
- `Intake.java:41-55`
- `Shooter.java:120-129`
- `Turret.java:233-234`
- `OpenLoopMotor.java:48-49`
- `VelocityControlledMotor.java:118`
- `ProfiledPositionMotor.java:116`

### Why this matters

Right now, the motor wrappers always call `motor.setPower(...)` even if the commanded value is identical to last loop.

This especially affects:

- intake, which writes one of four fixed powers every loop
- shooter follower motor, which mirrors the same power every loop
- turret manual mode, which can command the same output for many consecutive cycles

In FTC, motor writes are real outbound commands to the hub. If the value did not change, the command is often wasted bus traffic.

### Recommendation

Add command caching at the lowest reasonable layer, ideally `MetaMotor`.

Suggested pattern:

```java
private double lastAppliedPower = Double.NaN;

public void setPower(double power) {
    requireInitialized();
    double clipped = Range.clip(power, -maxPower, maxPower);
    if (Double.isNaN(lastAppliedPower) || Math.abs(clipped - lastAppliedPower) > 1e-4) {
        motor.setPower(clipped);
        lastAppliedPower = clipped;
    }
}
```

Also add a way to invalidate the cache when needed, for example after:

- `setMode(...)`
- encoder reset
- controller reset
- enabling/disabling a motor

### Why do it in `MetaMotor`?

Because then all wrappers benefit:

- `OpenLoopMotor`
- `VelocityControlledMotor`
- `ProfiledPositionMotor`
- intake
- shooter follower
- turret

### Expected impact

High on real hub traffic. This is one of the best "small code change, broad payoff" optimizations in your codebase.

## 4. Suppress identical servo writes too

### Where it happens

- LEDs: `Leds.java:286-289`
- gate: `Gate.java:14-18`
- hood: `Shooter.java:111-115`

### Why this matters

Servos are also being commanded repeatedly with the same target:

- `leds.setLeft(...)` / `setRight(...)` can repeatedly write the same color PWM
- `Gate.activate()` and `deactivate()` always write regardless of current state
- `Shooter.setHoodAngle(...)` writes immediately whenever called

The LED subsystem is especially likely to generate redundant writes because base colors are recomputed every loop.

### Recommendation

Cache the last commanded servo position per device and skip repeated `setPosition(...)` calls unless the target changed by some epsilon.

The LED subsystem can do this inside `apply(...)`. The gate can simply track a boolean or last position. The hood servo can track the last commanded angle.

### Expected impact

Medium. Not as important as drivetrain or shooter motor traffic, but still worthwhile because servo writes are frequent in your loop.

## 5. Move telemetry out of the hot path, or throttle it aggressively

### Where it happens

- `MainTeleOp.java:242-263`
- `Turret.java:98-100`
- `Turret.java:140-163`
- `Turret.java:176-187`
- `Turret.java:221-232`

### Why this matters

Your loop currently builds a large amount of telemetry every cycle:

- many booleans
- multiple currents
- multiple velocities
- pose objects
- follower data
- turret debug data

And `MainTeleOp.init()` sets the DS telemetry interval to `11 ms`:

- `MainTeleOp.java:79-80`

That is very aggressive. Even if the SDK limits actual transmission behavior internally, formatting all this data every loop still costs CPU, object creation, and string work.

### Recommendation

Split telemetry into modes:

- `competition` mode: only essentials, maybe `4-10 Hz`
- `debug` mode: full telemetry, only when tuning

Good candidates to throttle to slower rates:

- total current draw
- shooter current
- turret current
- pose pretty-printing
- Limelight diagnostic text

Simple strategy:

```java
if (loopCount % 5 == 0) {
    telemetryM.addData(...);
    telemetryM.update(telemetry);
}
```

or

```java
if (telemetryTimer.seconds() >= 0.1) {
    telemetryTimer.reset();
    telemetryM.update(telemetry);
}
```

### Important nuance

Telemetry is often one of the easiest ways to lose loop speed without realizing it, because the control logic still "looks simple" in code.

### Expected impact

High in practice, especially if you are tuning with many live fields and want max loops per second.

## 6. Stop allocating new controller state objects every loop

### Where it happens

- `VelocityControlledMotor.java:101-110`
- `ProfiledPositionMotor.java:99-117`
- `ProfiledPositionMotor.java:175-189`
- `TrapezoidalMotionProfileController.java`
- `MotionState.java`

### Why this matters

The motor package currently creates many small immutable objects in hot update paths:

- `MotionState`
- `Angle`
- `AngularVelocity`

Examples:

- `VelocityControlledMotor.update()` creates new target and current `MotionState`
- `ProfiledPositionMotor.update()` creates a new target state
- `ProfiledPositionMotor.readCurrentState()` creates another `MotionState`
- `TrapezoidalMotionProfileController.getReferenceState()` creates a new `MotionState`

On Android/ART, allocations in tight loops are not free. The Android performance docs explicitly recommend avoiding allocations in frequently executed loops when possible, because they increase GC pressure and can cause pauses or extra CPU work.

### Recommendation

You have three good options, ordered from least invasive to most aggressive:

1. cache reusable mutable state objects
2. convert inner controller math to primitive `double` fields instead of value objects
3. keep the public API expressive, but make the internal hot path allocation-free

For example, your internal controller update can use:

- `positionRad`
- `velocityRadPerSec`
- `accelRadPerSec2`

instead of building new `MotionState` objects every cycle.

### Expected impact

Medium. This is not as large as eliminating duplicate follower work or telemetry spam, but it improves GC behavior and makes loop timing more stable over a full match.

## 7. Cache reference states locally instead of rebuilding them repeatedly

### Where it happens

- `Turret.java:176-187`

### Why this matters

`turret.getReferenceState()` is called multiple times in one loop for telemetry:

- velocity
- position
- acceleration

If `getReferenceState()` constructs a new `MotionState` each call, you are paying repeated allocation and method overhead just for debug output.

### Recommendation

If you keep that telemetry:

```java
MotionState ref = turret.getReferenceState();
telemetryM.addData("ref vel", ref.getVelocity().toRadPerSec());
telemetryM.addData("ref pos", ref.getPosition().toRadians());
telemetryM.addData("ref a", ref.getAcceleration());
```

This is a smaller fix, but it is basically free to implement.

### Expected impact

Low to medium by itself, but good cleanup in a hot subsystem.

## 8. Avoid creating a new `Pose` every loop for moving-shot compensation

### Where it happens

- `Turret.java:107-118`

### Why this matters

`lookToGoalWhileMoving(...)` creates a new `Pose` on every call just to pass compensated coordinates into `lookToGoal(...)`.

That object is short-lived and contributes to allocation churn. It is not a major hardware bottleneck, but it is hot-loop allocation.

### Recommendation

Refactor `lookToGoal(...)` so it can accept primitive values:

- `x`
- `y`
- `heading`

Then compute the compensated target angle directly, without creating an intermediate `Pose`.

### Expected impact

Low to medium. Mostly a GC/cleanliness improvement.

## TeleOp-specific recommendations

## 9. Build a single per-loop snapshot right after clearing bulk cache

### Why this matters

The FTC SDK bulk-read sample makes an important point: the fast pattern is to read all needed inputs once at the beginning of the loop and reuse those stored values.

In your code, some values are read multiple times or by multiple subsystems during the same control cycle.

With manual bulk caching this is usually not extra hub traffic, but:

- the values are not fresher
- repeated subsystem reads make the loop harder to reason about
- you still pay Java-side method overhead

### Recommendation

Create a small `LoopInputs` snapshot containing things like:

- loop `dt`
- battery voltage
- IMU heading
- shooter measured velocity
- intake state
- Limelight validity / `tx`
- color sensor booleans
- follower pose / velocity after the one follower update

Then pass that snapshot into subsystems or at least into the parts of TeleOp that need it.

This gives you:

- fewer accidental duplicate reads
- more deterministic loop ordering
- easier profiling

## 10. Keep expensive diagnostics at a slower frequency than control

### Examples from current code

- `hardwareManager.getTotalCurrentDrawAmps()` in `MainTeleOp.java:251`
- `shooter.getCurrent1()` in `MainTeleOp.java:253`
- `follower.getPose()` string-heavy telemetry in `MainTeleOp.java:260-262`

### Recommendation

Run control every loop, but sample diagnostics at slower rates:

- current draw: `5-10 Hz`
- pretty telemetry / pose strings: `5-10 Hz`
- full debug dashboards: only when tuning

This keeps the control loop lean while preserving visibility.

## Motor-package recommendations

## 11. Consider offloading shooter velocity control to the hub as an A/B test

### Why this is worth testing

Your shooter currently uses a robot-controller-side velocity loop:

- read motor velocity
- compute PIDF in Java
- write power back to the hub every loop

That works, but it means shooter regulation speed is bounded by OpMode loop timing.

FTC hub firmware also supports onboard velocity control through encoder modes and motor PIDF support. The SDK release notes explicitly mention improved responsiveness for PIDF-based encoder speed control in supported motor modes.

### Recommendation

For the shooter only, test these two configurations:

1. current custom Java-side velocity loop
2. hub-side `RUN_USING_ENCODER` + `setVelocity(...)` with tuned coefficients

Measure:

- spin-up time
- recovery after a shot
- loop frequency of the whole OpMode
- shot-to-shot consistency

### Why only as an A/B test

Your current controller may still outperform the hub controller for your specific mechanism. But from a pure RC-loop-speed perspective, pushing regulation closer to the hardware can be a win.

## 12. Add command deadbands where full precision is not needed

### Where this helps

- power-write suppression
- servo-write suppression
- turret manual input

### Why this matters

If power changes by `1e-6`, sending a new hub command is pointless. Small floating-point noise can defeat write caching unless you use a tolerance.

### Recommendation

Use small epsilons:

- motor power: maybe `1e-4` to `1e-3`
- servo position: maybe `1e-4`

For manual turret input, consider zeroing tiny inputs near center if the gamepad path does not already do that.

## 13. Avoid repeated `Range.clip(...)` and repeated derived calculations when the configuration has not changed

### Where it happens

- `MetaMotor.setPower(...)`
- turret min/max/target tick conversions in `Turret.java:237-257`

### Why this matters

This is a smaller optimization, but when code is hot it is worth moving invariant math out of the hot path.

For example:

- min and max turret ticks can be precomputed once
- target conversion can be cached when the angle target changes
- clipped power can be cached with the write-suppression logic

### Expected impact

Low by itself, but good once the higher-level issues are fixed.

## Things that are probably not your main bottleneck

## 14. `Limelight3A.getLatestResult()` itself is probably cheap

The FTC SDK Javadoc describes `getLatestResult()` as returning the latest result already held by the Limelight driver. That suggests the RC-side call is not the same as performing a synchronous sensor transaction every loop.

So the bigger Limelight cost is usually:

- what you do with the result
- how much telemetry you generate from it
- the fact that vision changes robot behavior

not the Java getter itself.

This means I would not start by optimizing away the `getLatestResult()` call. I would optimize duplicate follower work, redundant writes, and telemetry first.

## 15. Repeated encoder reads inside one loop are less bad than repeated writes when using manual bulk caching

This is a subtle but important FTC point.

Because you already use manual bulk caching, repeated encoder reads in one loop usually come from the same cached bulk packet. So they are often not causing extra hub transactions.

That means in your codebase:

- repeated reads are still worth cleaning up for determinism and structure
- repeated writes are usually the bigger I/O target

This is why identical `setPower()` and `setPosition()` suppression is such a strong recommendation here.

## Suggested implementation order

If the only goal is maximum loops per second, I would do the work in this order:

1. remove or justify the second `follower.update()`
2. add write suppression to motors and servos
3. throttle telemetry heavily
4. centralize battery-voltage sampling
5. cache slow diagnostics like current draw
6. reduce hot-loop allocations in the motor package
7. test hub-side shooter velocity control as an A/B experiment

## Measurement plan

Do not rely on feel alone. Measure each change.

### Add loop segment timing

Track time spent in:

- `hardwareManager.update()`
- `follower.update()`
- `turret.loop()` or `turret.limelightAim(...)`
- `shooter.update()`
- `intake.update()`
- telemetry build + `telemetryM.update(telemetry)`

### Measure three key numbers

1. average loop time
2. 95th percentile loop time
3. worst-case loop time

Average loop time tells you overall speed. Worst-case and percentile timing tell you whether GC, telemetry, or a subsystem is causing stutters.

### Best quick A/B tests

Run these one at a time:

1. current code
2. telemetry nearly off
3. only one `follower.update()`
4. identical motor-write suppression
5. identical servo-write suppression
6. battery voltage cached at `10 Hz`

This will tell you very quickly which class of optimization matters most on your robot.

## Final recommendations

If you want the highest practical control rate on FTC hardware, I would focus on this principle:

> One loop should do one sensor snapshot, one control solve, and only the minimum required hardware writes.

Your code is already on the right track because it uses manual bulk caching. The next step is to make the rest of the architecture match that idea.

The most important concrete changes for this project are:

- eliminate the duplicate follower update
- suppress repeated motor and servo writes
- thin out telemetry aggressively
- centralize battery voltage and other slow-changing reads
- reduce per-loop object allocation in the motor stack

## Sources

Official FTC and Android sources used for this report:

- FTC SDK sample `ConceptMotorBulkRead`:
  https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
- FTC SDK repository and release notes:
  https://github.com/FIRST-Tech-Challenge/FtcRobotController
- FTC SDK `LynxModule` Javadoc:
  https://javadoc.io/static/org.firstinspires.ftc/Hardware/11.1.0/com/qualcomm/hardware/lynx/LynxModule.html
- FTC SDK `Limelight3A` Javadoc:
  https://javadoc.io/static/org.firstinspires.ftc/Hardware/11.1.0/com/qualcomm/hardware/limelightvision/Limelight3A.html
- FTC SDK `ServoImpl` Javadoc:
  https://javadoc.io/static/org.firstinspires.ftc/RobotCore/11.1.0/com/qualcomm/robotcore/hardware/ServoImpl.html
- Android performance guidance on allocations and GC:
  https://developer.android.com/topic/performance/vitals/render
  https://developer.android.com/topic/performance/memory-overview
