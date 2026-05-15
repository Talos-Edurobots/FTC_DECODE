# Turret Deep Dive From `9d04157` Through `HEAD`

This file is a turret-only guide for the changes from commit `9d0415770f1792e299707afd24d09ab16b64e5d4` through `HEAD` commit `c9637515d285fc263015668663fff4107bcb2cfb`.

The goal of this document is to let you read one file and recover:

- what changed in the turret code
- why those changes were made
- what control theory the code is using
- what problems the old design had
- what problems the new design solves
- what is still incomplete
- what future turret work was being set up

Where I am certain, I state things directly from the code and commit history.
Where I am inferring intent from commit messages, code structure, and the docs you added, I say so explicitly.

## Executive Summary

From `9d04157` onward, the turret evolved from a subsystem that already had goal-tracking behavior but still mixed control logic, telemetry, tuning, and manual/vision modes in a less structured way into a much more explicit control subsystem with:

- a dedicated motor abstraction (`MetaMotor` + `ProfiledPositionMotor`)
- explicit mechanism-angle limits
- proper motor-space vs mechanism-space conversion through the external gear ratio
- a clean split between two control modes:
  - motion-profiled position control for normal turret aiming
  - manual PID-style vision correction for Limelight aiming
- loop-time and battery-voltage awareness through `LoopState`
- explicit acceleration and later explicit deceleration limits in the motion profile
- better startup/reset behavior
- live configurable gains/profile limits
- centralized telemetry snapshots and telemetry modes

The deeper story is that this was not just "turret cleanup." It was a transition toward a more physically grounded turret controller and toward future SOTM work.

The controller itself became more principled. At the same time, your own documentation shows that you had already identified the next big limitation: the moving-shot target generation was still based on a heuristic pose shift instead of a physics-based angular lead and nonzero target velocity.

## The Turret Story in One Sentence

You were moving the turret from a mostly practical, tune-by-feel aiming system into a subsystem that separates:

- target generation
- motion planning
- motor control
- vision fallback
- observability

and that separation is exactly what enables the next wave of moving-shot improvements.

## Turret-Related Commits in Scope

These are the commits in the requested range that materially affected the turret:

1. `9d04157` - Refactor motor subsystems; remove `DriveTrain`
2. `ce3c7f0` - Add turret SOTM plan and controller docs
3. `fea45cb` - Add turret deceleration and refactor turret control
4. `c963751` - Add telemetry hub and provider system

Other commits in the range mattered indirectly because they changed teleop structure and diagnostics, but the four above are the real turret milestones.

## Before and After

## Before This Range

Before `9d04157`, the turret already existed and already did several important things:

- it could aim at the goal from robot pose
- it could apply a moving-shot heuristic using robot velocity
- it had Limelight aiming support
- it had manual power assistance
- it had safety concepts around motion and angle bounds

But the implementation history before this range suggests the turret had grown organically over many iterations:

- manual turret control commits
- safety check commits
- angle normalization fixes
- moving-shot additions
- power ramping additions
- Limelight alignment work

That usually produces a system that works, but whose responsibilities are not perfectly separated.

## After This Range

By `HEAD`, the turret is much more formalized:

- hardware is represented explicitly through `MetaMotor`
- normal aiming uses a `ProfiledPositionMotor`
- profile coefficients are scaled into proper units through `MotorCoefficientScaler`
- angle units are handled explicitly via `EncoderConverter` and `Angle`
- loop timing and battery compensation are fed in through `LoopState`
- control mode switching is explicit and reset-aware
- motion profile tuning supports asymmetric accel/decel
- expensive telemetry is throttled and structured

That is a meaningful architectural jump.

## The Core Design You Ended Up With

The turret at `HEAD` has two different control paths.

### 1. Normal aiming path: profiled position control

This path is used when the turret is asked to:

- `faceForward()`
- `setAngleRadians(...)`
- `lookToGoal(...)`
- `lookToGoalWhileMoving(...)`

In this path, the turret does not directly command raw motor power from target error.

Instead it does this:

1. Decide the desired mechanism angle.
2. Clip that angle to safe mechanism limits.
3. Convert mechanism angle into motor-space angle using the external gear ratio.
4. Feed the target into a motion-profiled controller.
5. Let the controller generate a reference position, velocity, and acceleration over time.
6. Let the PIDF law convert that reference into motor power.

That is a big conceptual improvement over a raw "error to power" control loop.

### 2. Vision path: manual PID-style correction

This path is used during `limelightAim(...)`.

Instead of letting the profiled controller chase an angle target derived from geometry, the code:

- reads Limelight horizontal offset `tx`
- converts `tx` into an error signal
- computes output with a manual PID-like law
- applies ramping and angle-limit guards
- allows a small manual driver bias through `manualExtraPower`

This path exists because Limelight aiming is not the same problem as normal turret position control.

In Limelight mode, the vision signal itself is the error measurement. You are effectively trying to zero image-plane error, not just follow a preplanned pose-derived angle.

## Why Two Modes Make Sense

This split is important and was one of the most meaningful improvements in the range.

Normal turret aiming and Limelight alignment are related, but they are not identical problems:

- normal aiming is a geometric target-tracking problem
- Limelight aiming is an image-error servo problem

If you try to make one controller shape solve both in the same way, you usually end up with awkward compromises:

- geometry-based aiming can feel too indirect for vision lock
- vision-based correction can feel noisy or unstable for normal target positioning

By separating the modes, you kept the normal path smooth and motion-limited while letting the vision path stay reactive.

## Commit `9d04157`: The Big Turret Refactor

This is the foundation for everything that follows.

## What changed

This commit replaced a looser turret/motor setup with a more explicit one built around:

- `MetaMotor`
- `ProfiledPositionMotor`
- `MotionProfilingCoefficients`
- `EncoderConverter`
- `LoopState`

It also moved more turret definition into `RobotConstants`:

- motor name
- motor type
- direction
- zero-power behavior
- external gear ratio
- profile coefficients
- current limits
- min/max mechanism angles

## Why this change mattered

This solved several structural problems at once.

### Problem 1: hidden hardware assumptions

Older turret code likely depended on more implicit configuration behavior living inside `MotorConfig`.

By making turret hardware parameters explicit in `RobotConstants`, the code became easier to reason about:

- what motor it is
- what gear ratio it uses
- what angle range is valid
- what motion limits apply

That matters because turret behavior is extremely sensitive to unit mistakes and configuration drift.

### Problem 2: mechanism-space vs motor-space confusion

The turret has an external gear ratio of `2.8`.

That means the motor rotates `2.8` radians for every `1` radian of mechanism rotation.

If that mapping is not handled consistently, you get errors like:

- wrong target positions
- wrong velocity limits
- wrong acceleration limits
- confusing telemetry
- tuning that only "works" because multiple mistakes cancel each other

The refactor made that conversion explicit:

- mechanism-side commands stay in mechanism angle units
- motor-side internals are converted through `toMotorRadians(...)`
- profile coefficients are scaled using `MotorCoefficientScaler`
- telemetry later reports both measured and reference quantities in more intentional ways

This is one of the deepest quality improvements in the turret work.

### Problem 3: normal aiming needed a true profiled control path

The commit message explicitly says turret control was reworked to support:

- profiled position control
- manual PID fallback
- loop timing and battery compensation

This means you were trying to solve a real control-quality issue:

- raw angle commands can be too aggressive
- direct PID without profile shaping can overshoot or jerk
- battery sag can make identical tuning behave differently over a match
- inconsistent loop time can destabilize derivative or feedforward terms

The profiled controller addresses that by turning the aiming problem into:

"Follow a smooth reference trajectory to the target" instead of "slam straight toward target error."

### Problem 4: telemetry was too tangled with control logic

At this stage telemetry was still local to the turret class, but the refactor already updated what the turret reported:

- power
- velocity
- reference velocity
- reference position
- reference acceleration
- target
- current
- angle bounds

That suggests you were actively debugging controller behavior, not just target angle.

This is a sign that you had moved from "does it turn?" to "is the controller doing the right thing internally?"

## The Control Theory Introduced Here

The normal aiming path after this refactor is a motion-profiled PIDF position controller.

In practice, that means:

1. The turret receives a target angle.
2. It does not instantly try to jump there at arbitrary speed.
3. It creates a reference path limited by:
   - max velocity
   - max acceleration
4. The PIDF controller tracks that moving reference.

So the controller is not only answering:

"How far am I from the final target?"

It is also answering:

"How fast should I be moving right now if I want to arrive smoothly and within physical limits?"

This is exactly the right direction for a turret that needs to feel:

- fast
- smooth
- repeatable
- mechanically safe

## Why `LoopState` and Battery Compensation Were Important

This is easy to overlook, but it was one of the smartest changes.

The turret now updates using loop timing and battery scaling information.

That matters because FTC loops are not perfectly constant.

If `dt` changes from loop to loop:

- derivative-like terms can spike
- reference integration can drift
- feedforward can misbehave

If battery voltage drops:

- the same requested power produces less physical response

By feeding `dt` and a battery-voltage factor into the motor-control path, you were trying to make controller behavior more invariant to real match conditions.

In other words:

- same code
- same target
- same tuning

should behave more similarly across different loop rates and battery states.

That is a very real turret-control improvement, not just a code cleanup.

## The Manual PID Limelight Fallback

`9d04157` also formalized the Limelight path.

In `limelightAim(...)`, the code:

- switches into `MANUAL_PID`
- resets controller state when entering that mode
- computes an error from Limelight `tx`
- uses `kp`, `ki`, `kd`, and `ks`
- ramps power changes through `rampPower(...)`
- blocks unsafe output when angle limits are exceeded
- allows a manual bias if the visual error is small

This tells a clear story about the problem you were solving.

### Why not use the profiled controller directly for Limelight?

Likely reasons, based on the code and commit text:

- Limelight `tx` is already an error signal in image space
- you wanted quick correction rather than a long trajectory plan
- you needed a fallback that could still be shaped by ramping and bounds
- you wanted to reuse familiar PID-style tuning during visual lock

This is a pragmatic hybrid:

- profile the normal motion problem
- use a more direct servo loop for visual centering

That is a sensible tradeoff.

## What the Limelight path was protecting against

The manual PID path includes:

- output ramping
- integration only under certain conditions
- lower/upper mechanical bound checks

Those features strongly suggest the older or simpler approach had likely encountered some combination of:

- jerky response
- sudden polarity changes
- risk of driving into hard stops
- unstable behavior when `tx` became noisy or zeroed

The refactor made those failure modes more explicit and manageable.

## Commit `ce3c7f0`: The Turret Theory and SOTM Documentation

This commit did not change turret runtime code directly, but it is crucial for understanding your intent.

You added:

- `doc/TurretController.md`
- `TeamCode/.../TurretSOTMImplementationPlan.md`

These files explain two important things.

## First: you believed the core profiled controller was already fundamentally sound

That is a very important conclusion.

Your docs do not say:

- "rewrite the PID"
- "throw out the controller"
- "the turret power law is wrong"

Instead, the docs argue that the controller already has the right shape:

- position correction
- velocity-aware behavior
- feedforward terms
- motion profiling

This is a sign of engineering maturity: you had separated controller quality from target-generation quality.

## Second: you identified the real remaining weakness as moving-shot targeting

The documents make the future plan very explicit:

- the current moving-shot method is heuristic
- it uses a pose shift based on full velocity
- that is not physics-based
- it mixes radial and tangential motion
- it does not explicitly model projectile flight time
- it does not provide a nonzero target velocity for the turret to track during motion

This is probably the single most important "why" behind the later turret work.

You were not just tuning the turret because it felt imperfect.
You had identified a deeper conceptual issue:

the controller could be fine, while the target fed into it could still be wrong.

## The Moving-Shot Problem in Plain English

The current moving-shot logic in `lookToGoalWhileMoving(...)` does:

`compensatedPose = pose + leadFactor * velocity * distance`

Then it aims as if the robot were at that compensated pose.

This is clever, compact, and usable, but it is only a heuristic.

### Why it is not physically ideal

Only the tangential component of robot velocity directly changes the required aim angle to a fixed goal.

Radial velocity mostly changes:

- distance
- therefore shot timing and ballistics

but not bearing in the same direct way.

By using the full velocity vector, the heuristic can:

- overreact to radial motion
- underreact to tangential motion
- couple distance and angle compensation in a way that is hard to tune globally

Your docs correctly identify that the better solution is:

- compute tangential velocity explicitly
- estimate projectile flight time
- convert that into an angular lead
- ideally feed a nonzero target angular velocity into the turret profile

That shows where your turret work was heading.

## Commit `fea45cb`: Explicit Deceleration and Control Refinement

This is the next major runtime improvement.

## What changed

This commit added:

- explicit `maxDeceleration`
- live profile coefficient updates
- cleaner turret startup/reset behavior
- safer `dt` handling
- better unit conversion
- updated turret tuning values

This is where the turret control became more refined rather than merely restructured.

## Problem 1: acceleration and deceleration were previously forced to be the same

Before this commit, the profile logic effectively assumed one acceleration limit did both jobs:

- speeding up
- slowing down

That is often acceptable at first, but it is rarely ideal on a real mechanism.

Why?

- a turret may need aggressive acceleration to feel responsive
- but gentler or at least independently tuned deceleration to avoid overshoot or harsh stopping
- or the reverse, depending on inertia, friction, and gearing

The change to `TrapezoidalMotionProfileController` is small in code but significant in behavior:

- stopping distance now uses `maxDeceleration`
- accelerating toward target uses `maxAcceleration`
- braking behavior is no longer assumed to match acceleration behavior

This is a real control upgrade.

## Why that matters physically

Stopping distance is:

`v^2 / (2a)`

If the value used in that calculation is wrong, the controller can brake:

- too late and overshoot
- too early and feel sluggish

Adding explicit deceleration gives you a separate knob for how early and how hard the turret should plan to brake.

For a geared turret, this matters a lot because:

- inertia is nontrivial
- backlash and friction can distort low-speed response
- aiming quality near the goal depends heavily on how the profile enters the stopping phase

## Problem 2: the profile and gains needed to be live-configurable

`fea45cb` added:

- setters in `MotionProfilingCoefficients`
- setters in `PIDFFCoefficients`
- `ProfiledPositionMotor.setMaxPower(...)`
- `Turret.applyProfileConfigurables()`

That means the turret no longer treated profile values as static boot-time constants only.

Instead, each loop can re-apply:

- `kp`
- `ki`
- `kd`
- `ks`
- `kv`
- `ka`
- `maxVel`
- `maxAcc`
- `maxDec`
- `maxPower`

This is not just convenience. It suggests a real need:

- tuning from configurables/debugger
- quick iteration on-field
- changing values without rebuilding the whole controller stack

It also explains why the code was being pushed toward clearer unit scaling and clearer telemetry: live tuning is only useful if the numbers mean what you think they mean.

## Problem 3: startup and mode transitions needed to be cleaner

The commit added a dedicated `start()` method to the turret that:

- resets the controller
- clears manual integral state
- clears the last manual error
- resets the loop timer

This suggests you were solving a subtle but real issue:

- stale controller state from init or previous mode
- derivative spikes on first loop
- manual-mode memory carrying into profiled mode or vice versa

Those issues are common in mechanisms that can switch between autonomous geometry tracking and vision/manual correction.

The addition of `start()` indicates you wanted the turret to begin teleop in a known dynamic state, not just in a known position target state.

## Problem 4: bad loop `dt` needed guarding

This commit added `MAX_REASONABLE_LOOP_DT_SECONDS = 0.25` and logic that turns obviously bad `dt` values into a zero-`dt` update path.

That is very revealing.

It means you had probably seen or anticipated one of these:

- first-loop timing spikes
- long pauses causing giant derivative or profile jumps
- debugger-induced stutters
- mode-transition timing artifacts

Without guarding bad `dt`, a profiled controller can produce nonsense:

- huge reference velocity step
- huge derivative contribution
- ugly power burst

By explicitly zeroing out unreasonable `dt`, the code becomes more resilient to real FTC runtime conditions.

## Problem 5: unit clarity needed improvement

`fea45cb` also improved how telemetry and profile state used `EncoderConverter`.

Instead of mixing raw radian values and tick values loosely, the code intentionally converts:

- reference velocity to ticks per second
- reference acceleration to ticks per second squared
- reference angle to ticks where helpful

This matters because tuning sessions often fail not because the controller is wrong, but because the telemetry is hard to interpret.

You were making the internal state more debuggable.

## Tuning changes in `RobotConstants`

This commit changed turret constants significantly.

Earlier turret values:

- profile in `TURRET_PROFILE_COEFFICIENTS`:
  - `maxVel = 1800`
  - `maxAcc = 4500`
  - decel implicitly same until later support was added
- `TURRET_CONFIG` PIDF:
  - `kp = 0.068`
  - `ki = 0`
  - `kd = 0.002`
  - `ks = 1.2`
  - `kv = 0.005687094208999908`
  - `ka = 0.0004`

After `fea45cb`, `TURRET_CONFIG` moved to:

- motion profile:
  - `maxVel = 1800`
  - `maxAcc = 3000`
  - `maxDec = 2000`
- PIDF:
  - `kp = 0.005`
  - `ki = 0`
  - `kd = 0`
  - `ks = 0.3`
  - `kv = 0.005687094208999908`
  - `ka = 0.0004`

This is a large behavioral retune.

## What that retune likely means

This part is inference, but it is strong inference.

The changes suggest you were trying to make the turret:

- less aggressive in proportional correction
- less reliant on derivative action in the config path
- less static-friction-heavy
- more profile-dominated rather than correction-dominated
- gentler in accel/decel behavior

In other words, the controller may previously have been too "muscular" and not smooth enough, or at least too sensitive to the relationship between tuned gains and actual profiled motion.

That fits very well with the rest of the commit:

- explicit decel
- safer dt
- live tuning
- cleaner startup

Everything points toward stabilization and refinement, not feature expansion.

## Commit `c963751`: Telemetry Becomes a System

This is not purely a turret control change, but it significantly improves turret understanding and tuning.

## What changed

The turret now implements `TelemetryProvider`.
It gained:

- `getTelemetrySnapshot(...)`
- `collectTelemetry(...)`
- throttled current sampling
- categorized telemetry by mode and cost

At the same time, `MainTeleOp` started:

- registering the turret with a `TelemetryHub`
- publishing telemetry through modes
- allowing telemetry mode cycling

## Why this matters for turret control

Turret tuning is impossible if you cannot clearly see:

- target
- measured angle
- velocity
- reference velocity
- reference acceleration
- power
- current
- overcurrent state
- limit state
- active gains/profile limits

The earlier local telemetry served the same purpose, but the new framework makes it intentional:

- competition mode can stay lightweight
- debug mode can expose more internals
- trace mode can expose static tuning data
- expensive values can be throttled

This reflects a shift in engineering style:

telemetry is no longer an ad hoc printout; it is now part of the control system design.

That matters because the turret had become one of the most sophisticated mechanisms in the project.

## The Final Turret Architecture at `HEAD`

At the end of the range, the turret subsystem has these main responsibilities.

### 1. Hardware definition

- motor name, type, direction, limits from `RobotConstants`
- hardware binding through `MetaMotor`

### 2. Geometric aim generation

- `lookToGoal(...)`
- `lookToGoalWhileMoving(...)`

### 3. Mode management

- `PROFILED`
- `MANUAL_PID`

### 4. Motion-limited normal aiming

- target angle clipping
- gear-ratio conversion
- profile generation
- profiled PIDF tracking

### 5. Vision-based correction

- Limelight `tx` closed-loop correction
- manual output shaping
- limit protection

### 6. Runtime robustness

- loop `dt` measurement
- battery compensation
- startup reset
- safe control-mode transitions

### 7. Observability

- telemetry snapshots
- reference vs measured data
- throttled expensive sensors
- telemetry modes

That is a healthy subsystem boundary.

## Detailed Control Theory of the Current Turret

## Angle generation

The stationary target angle is:

`atan2(goalY - robotY, goalX - robotX) - robotHeading`

Then the angle is normalized on the positive side by subtracting `2pi` if needed and clipped into:

- min angle: `-100 deg`
- max angle: `120 deg`

This ensures the mechanism is commanded only within safe travel.

## Gear-ratio conversion

The target angle you think about as a turret angle is not the same as the angle the motor must rotate.

Because:

`motorAngle = mechanismAngle * 2.8`

All profile internals must therefore operate consistently in motor-side units or use converted coefficients.

The code handles that explicitly now.

## Motion profile behavior

The profiled controller carries a reference state:

- reference position
- reference velocity
- reference acceleration

Each loop it decides whether to:

- accelerate toward the target
- or decelerate so it can stop cleanly

With `fea45cb`, that deceleration decision became more physically correct because stopping distance uses `maxDeceleration`.

This is one of the most important control-quality upgrades in the range.

## PIDF behavior

The profiled path uses a PIDF structure.

Conceptually the controller combines:

- position error correction
- dynamic matching to the moving reference
- static friction feedforward
- velocity feedforward
- acceleration feedforward

This is exactly what you want for a turret that is not just moving to a point but following a planned state trajectory.

## Battery scaling

Battery compensation means feedforward behavior is less dependent on battery sag.

That is important because feedforward terms are trying to model expected physical effort.
Without voltage awareness, that model gets worse as the battery drops.

## Limelight manual PID behavior

The Limelight path is a direct servo loop on image-plane error:

- error = `-tx`
- derivative = change in error over time
- integral accumulates under a guarded condition
- `ks` gives a directional static-friction kick
- output ramping limits abrupt changes
- near-zero error returns control to `manualExtraPower`

This path is a practical "fast correction with safeguards" design.

## The Real Problems You Were Facing

Based on the code and documents, the turret work in this range seems to have been responding to five main classes of problems.

## 1. Architectural coupling

Before the refactor, turret behavior, hardware setup, legacy config, telemetry, and control-state concerns were more entangled.

You addressed that by making:

- constants explicit
- control modes explicit
- hardware abstractions explicit
- telemetry structure explicit

## 2. Unit and scaling ambiguity

Turret mechanisms are especially vulnerable to silent unit errors because:

- they are rotational
- they are geared
- they are often tuned in one unit system and measured in another

Your use of `EncoderConverter`, `MotorCoefficientScaler`, and explicit motor/mechanism angle conversion is a direct response to that risk.

## 3. Dynamic inconsistency

The addition of `LoopState`, battery scaling, `start()`, and `dt` guards strongly suggests that controller behavior needed to become more consistent under real runtime conditions.

This is often what separates a controller that "usually works" from one that behaves repeatably on-field.

## 4. Stopping behavior and overshoot control

The explicit deceleration work strongly indicates the old symmetric profile was not expressive enough.

This is usually caused by one of:

- overshoot near target
- harsh braking
- sluggish stop tuning
- inability to tune entry and exit phases independently

Adding `maxDeceleration` is exactly the right response.

## 5. Moving-shot correctness

Your own docs make this crystal clear:

the next limiting factor was not basic controller architecture, but the way moving-shot lead was generated.

The current heuristic was good enough to exist, but not good enough to be the final design.

## What Changed in TeleOp Use of the Turret

During this range, turret use inside teleop also became cleaner.

The current `MainTeleOp` flow is:

- create the turret subsystem
- `init()` it during teleop init
- `start()` it during teleop start
- each loop:
  - apply manual control from triggers
  - if Limelight aiming is enabled, use `limelightAim(result)`
  - otherwise use `lookToGoalWhileMoving(...)` and `loop()`

This is a clean and readable operational model.

It also shows the intended runtime concept:

- the driver can add manual bias
- vision mode and geometry mode are mutually exclusive
- moving-shot support is the default non-Limelight target path

## What Is Still Incomplete at `HEAD`

Your turret is much better at `HEAD`, but your own docs show that the architecture is still in transition toward a better SOTM design.

The biggest remaining gap is this:

## 1. Moving-shot lead is still heuristic

`lookToGoalWhileMoving(...)` still uses:

- full velocity vector
- magic `movingShotLeadFactor`
- pose translation instead of angular-lead computation

So although the controller beneath it is stronger, the moving-shot target is still not fully physics-based.

## 2. The turret profile still aims for a position target rather than an explicit moving-shot target state

Your docs argue that true SOTM should also track:

- nonzero target angular velocity

That is not yet implemented in the current code path.

This matters because a moving target in robot-local coordinates should not always be treated as "go to this angle and settle."
Sometimes the correct target is "be at this angle and keep sweeping at this angular velocity."

## 3. Flight time is not yet integrated into the live aiming path

The current moving-shot heuristic does not use explicit projectile flight time.

That means it cannot naturally scale lead in the most physically meaningful way across:

- short shots
- long shots
- different robot speeds

## The Future Plan Your Docs Point Toward

If I compress your own turret planning documents into the most important future work, it is this:

### 1. Replace pose-shift moving-shot lead with angular lead

Instead of faking a shifted pose, compute:

- vector to goal
- tangential velocity
- projectile flight time
- lead angle directly

### 2. Separate tangential and radial velocity

Only tangential motion should dominate bearing correction.
Radial motion should influence distance and timing, not be mixed blindly into angle lead.

### 3. Feed nonzero compensation velocity into the turret target state

For SOTM, the turret should not always try to come to rest at the target.
It should sometimes intentionally maintain angular velocity.

### 4. Keep the profiled PIDF architecture

Your docs strongly suggest this should be evolutionary, not revolutionary.

That means:

- keep the controller core
- improve the target-generation layer
- improve target-state richness

This is the right plan.

## Best Interpretation of Why You Made These Changes

If I had to summarize the intent behind the turret work as accurately as possible:

You were no longer satisfied with a turret that merely points roughly correctly.
You were building a turret subsystem that could be:

- tuned systematically
- debugged systematically
- trusted across battery and loop variations
- extended toward better moving-shot physics

That is why the changes span:

- code architecture
- control theory
- telemetry
- tuning APIs
- motion profiling math
- future design documents

It was a deliberate foundation-building phase.

## Most Important Files To Read If You Want To Cross-Check This Doc

If you want to verify this deep dive against the code, the best files are:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/TrapezoidalMotionProfileController.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TurretTelemetrySnapshot.java`
- `doc/TurretController.md`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/TurretSOTMImplementationPlan.md`

## Final Bottom Line

From `9d04157` to `HEAD`, the turret changes were not random tuning edits.

They were a coherent progression:

1. move the turret onto a cleaner motor/control abstraction
2. make units, limits, and gear-ratio handling explicit
3. split normal aiming from Limelight image-error control
4. make the profile timing and battery aware
5. make stopping behavior more tunable with explicit deceleration
6. expose internal state well enough to debug the controller, not just the target
7. document the next major step: replacing heuristic moving-shot lead with physics-based SOTM targeting

If you want the single most important insight from this whole range, it is this:

the controller became much more solid, and the next frontier you identified was not "more PID tuning" but "better target generation for moving shots."
