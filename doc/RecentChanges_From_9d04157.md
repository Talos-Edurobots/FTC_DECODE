# Guide to Changes From Commit `9d04157` Through `HEAD`

This guide summarizes everything from commit `9d0415770f1792e299707afd24d09ab16b64e5d4` through the current `HEAD` commit `c963751`.

If your goal is "what did I actually change recently?", the short version is:

- You replaced older motor/subsystem patterns with the newer motor facade and profiling APIs.
- You removed the old `DriveTrain` subsystem from teleop initialization.
- You consolidated duplicated teleop code into a new `MainTeleOp` class.
- You spent a lot of effort on turret control, tuning, documentation, and telemetry.
- You refactored LEDs into a much more capable stateful subsystem.
- You added a centralized telemetry framework and moved major subsystems onto it.
- You also added several large design/review documents to explain the architecture and future work.

## Timeline

| Date | Commit | Summary |
|---|---|---|
| 2026-05-12 | `9d04157` | Refactor motor subsystems; remove `DriveTrain` |
| 2026-05-12 | `6d1fc8e` | Debugger cleanup, `MotorConfig.copy()`, helper utilities |
| 2026-05-12 | `5991288` | Extract shared teleop flow into `MainTeleOp` |
| 2026-05-12 | `ce3c7f0` | Add turret controller and SOTM planning docs |
| 2026-05-13 | `3070115` | Major LED refactor, color sensor throttling, more docs |
| 2026-05-14 | `fea45cb` | Add turret deceleration support and refine turret control |
| 2026-05-14 | `c963751` | Add telemetry hub/provider system |

## Big Picture

Across these seven commits, the codebase changed in four main ways:

1. Motor and subsystem architecture became more explicit.
   You moved subsystems onto `MetaMotor`, `OpenLoopMotor`, `VelocityControlledMotor`, and `ProfiledPositionMotor` style APIs, and you pushed more robot-specific setup into `RobotConstants`.

2. Teleop structure became centralized.
   `MainBlue` and `MainRed` stopped containing their own large control loops and became thin wrappers around `MainTeleOp`.

3. Turret control became the main focus area.
   You reworked how the turret is initialized, profiled, tuned, manually controlled, documented, and reported through telemetry.

4. Telemetry and loop efficiency became intentional systems.
   You added telemetry modes, provider registration, throttled sampling, color sensor throttling, and several design docs around loop and telemetry performance.

## Commit-by-Commit

### 1. `9d04157` - Refactor motor subsystems; remove `DriveTrain`

This is the architectural starting point for the range.

What changed:

- Deleted `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/DriveTrain.java`.
- Removed `DriveTrain` initialization/import usage from `MainBlue` and `MainRed`.
- Expanded `RobotConstants` so subsystem hardware setup is more explicit:
  - motor names
  - motor types
  - directions
  - limits
  - turret profile coefficients
  - turret angle bounds
- Migrated subsystem code to the newer motor abstraction layer:
  - `Hang`
  - `Intake`
  - `Shooter`
  - `Turret`
- Reworked `Turret` heavily:
  - profiled position control became the main path
  - manual PID fallback was added
  - loop timing and battery compensation started using `LoopState`
  - telemetry fields were updated

Why it matters:

- This commit is where the code stops being "legacy drivetrain plus older subsystem patterns" and starts using the newer motor framework more consistently.
- If recent bugs or behavior changes started around turret motion, subsystem initialization, or motor tuning, this is the first commit to inspect.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java`

### 2. `6d1fc8e` - Debugger cleanup, `MotorConfig.copy()`, helper utilities

This commit is mostly about safer configuration handling and better debugging support.

What changed:

- Added `MotorConfig.copy()`.
- Updated debugger OpModes so they clone configs instead of mutating shared `RobotConstants`.
- Added helper logic to centralize:
  - battery voltage reading
  - legacy loop `dt` handling
- Cleaned up debugger behavior around:
  - Limelight null handling
  - servo target initialization
  - clamping
  - intake/shooter/turret initialization
  - telemetry quality
- Added `doc/DebuggerReview.md`.

Why it matters:

- Before this, tweaking a debugger config could unintentionally mutate shared robot constants.
- This commit makes experimentation safer and reduces "debug mode changed production behavior" risk.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/Debugger.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/MotorConfig.java`
- `doc/DebuggerReview.md`

### 3. `5991288` - Extract shared teleop flow into `MainTeleOp`

This commit is mostly a structural refactor rather than a brand-new feature.

What changed:

- Added `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java`.
- Replaced the large duplicated logic inside `MainBlue` and `MainRed` with thin wrappers that delegate to `MainTeleOp`.
- Centralized shared behavior such as:
  - hardware initialization
  - Limelight pipeline selection by alliance
  - intake/shooter/turret/LED flow
  - telemetry handling
- `stop()` now clears follower/alliance blackboard entries.

Why it matters:

- This is the key maintainability commit in the range.
- After this point, teleop behavior differences between red and blue should mostly come from configuration and alliance selection, not copy-pasted logic drift.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainBlue.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainRed.java`

### 4. `ce3c7f0` - Add turret controller and SOTM planning docs

This commit does not change runtime code directly. It adds documentation.

What changed:

- Added `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/TurretSOTMImplementationPlan.md`.
- Added `doc/TurretController.md`.

What the docs cover:

- the current profiled turret controller
- moving-shot shortcomings
- angular lead concepts
- flight-time-based compensation
- telemetry ideas
- phased implementation advice
- safety checks and validation steps

Why it matters:

- This explains the reasoning behind later turret commits.
- If you want to remember not just what changed, but what direction you were aiming for, this is the best place to read that intent.

### 5. `3070115` - Major LED refactor, sensor throttling, loop/telemetry docs

This is one of the biggest commits by line count.

What changed in code:

- Refactored `Leds` into a stateful subsystem with:
  - `Side` enum
  - base colors
  - current colors
  - blink / pulse / hold / rgb behavior
  - effect lifecycle management
  - optional telemetry
- Centralized LED updates through `update(dt)`.
- Added LED demo support in `Debugger`.
- Extended `ColorSensors` with throttled updates and helpers:
  - configurable update rate
  - `forceUpdate()`
  - `setUpdateHz()`
  - `getUpdateHz()`
- Removed a duplicated `follower.update()` call from `MainTeleOp`.

What changed in docs:

- Added `doc/FTC_Loop_Optimization_Report.md`
- Added `doc/FTC_Telemetry_Management_Plan.md`

Why it matters:

- LEDs went from direct/instant servo writes toward a small effect engine.
- Sensor polling and telemetry volume started being treated as loop-performance concerns instead of incidental implementation details.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Leds.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/ColorSensors.java`
- `doc/FTC_Loop_Optimization_Report.md`
- `doc/FTC_Telemetry_Management_Plan.md`

### 6. `fea45cb` - Add turret deceleration support and refine turret control

This is the second major turret-control commit.

What changed:

- Added explicit `maxDeceleration` support across the profiling stack:
  - `MotorConfig`
  - `MotionProfilingCoefficients`
  - `MotorCoefficientScaler`
  - `ProfiledPositionMotor`
  - trapezoidal controller math
- Added setters/helper methods for tuning APIs.
- Updated turret constants and profile values in `RobotConstants`.
- Refactored TeleOp/debug usage to go through the newer `Turret` subsystem path.
- Improved turret startup/reset behavior and unit conversion handling.
- Disabled several test OpModes to reduce runtime menu clutter.

Why it matters:

- This changes motion-profile behavior in a real way, not just structure.
- If the turret started braking differently, overshooting less, or feeling safer/more controlled, this commit is likely why.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/coefficients/MotionProfilingCoefficients.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/TrapezoidalMotionProfileController.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java`

### 7. `c963751` - Add telemetry hub/provider system

This is the latest commit in the range and one of the most important architectural changes.

What changed:

- Added a centralized telemetry package:
  - `TelemetryHub`
  - `TelemetryCollector`
  - `TelemetryProvider`
  - `TelemetryMode`
  - `TelemetryPublishPolicy`
  - `TelemetryCostClass`
  - `ThrottledValue`
- Converted `MainTeleOp`, `Shooter`, and `Turret` to participate in this telemetry system.
- Added telemetry snapshot objects for shooter and turret.
- Replaced many direct `telemetry.addData(...)` style writes with centralized publish logic.
- Added mode cycling from gamepad and support for:
  - competition/debug/trace-style visibility
  - throttled expensive values
  - mode-entry-only publication for static fields

Why it matters:

- Telemetry is no longer just "print whatever right now".
- The code now treats telemetry as a managed system with cost awareness and different visibility levels.
- This should make it easier to keep competition telemetry useful without flooding loop time.

Files worth reading:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryHub.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryCollector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java`

## What Actually Changed Most

If you ignore docs and look at practical impact, the biggest recent changes were:

### Highest impact on robot behavior

- `Turret.java`
  - new control paths
  - profiling changes
  - safer loop-state handling
  - new telemetry snapshots
- `MainTeleOp.java`
  - teleop flow centralization
  - alliance-specific setup
  - telemetry mode cycling
  - intake / LEDs / turret coordination
- `RobotConstants.java`
  - updated tuning values and more explicit motor configuration
- `Leds.java`
  - behavior changed from simple direct writes to managed effects

### Highest impact on maintainability

- `MainTeleOp.java` extraction
- `TelemetryHub` and friends
- `MotorConfig.copy()`
- removal of `DriveTrain`

### Highest impact on documentation / future direction

- `doc/TurretController.md`
- `TeamCode/.../TurretSOTMImplementationPlan.md`
- `doc/FTC_Loop_Optimization_Report.md`
- `doc/FTC_Telemetry_Management_Plan.md`

## Fast Reading Order

If you only want to spend 10 to 15 minutes, read in this order:

1. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java`
   This shows the current teleop control flow and how subsystems now fit together.

2. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
   This is the subsystem with the heaviest recent change volume and the most behavior risk.

3. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryHub.java`
   Then glance at `TelemetryCollector.java` so you understand the new telemetry architecture.

4. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Leds.java`
   This is the other subsystem that changed a lot behaviorally.

5. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java`
   This gives the latest tuning/config values that many of the other changes depend on.

If you want the design intent after that, read:

- `doc/TurretController.md`
- `doc/FTC_Telemetry_Management_Plan.md`

## Cumulative Size of the Change Range

From `9d04157^` through `HEAD`:

- 40 files changed
- about 4,423 insertions
- about 894 deletions

That means this was not a small cleanup. It was a real architecture-and-control pass, especially around turret behavior, teleop structure, LED control, and telemetry.

## Bottom Line

If you want the most honest one-paragraph summary:

You spent these commits modernizing subsystem control around the newer motor abstractions, removing old teleop/drivetrain duplication, deepening turret control and profiling work, turning LEDs into a proper effect subsystem, and formalizing telemetry/loop-efficiency as first-class architecture concerns. The most important runtime changes are in `MainTeleOp`, `Turret`, `Leds`, `RobotConstants`, and the new telemetry package; the rest mostly supports those changes or documents where the system is going next.
