# FTC Judging Presentation Notes - Software / Control

## What Judges Are Looking For

From the 2025-2026 FTC judging summary, the Control Award is not just "say advanced programming words." Judges are checking:

- What hardware or software control components are on the robot.
- What challenge each component solves.
- How each component works.
- Whether the robot uses external feedback and control.
- Whether the control solution works consistently in matches.
- How reliability was considered.
- What engineering process and lessons learned led to the final system.

So the speech should follow:

Problem -> what we changed -> why it helped.

Detailed math belongs in Q&A unless a judge asks.

## Main Message

Our software reduces driver workload during the scoring cycle. The driver chooses where to drive and when to collect or shoot. The robot handles the repeated timing-sensitive work: corrected pose, turret aim, shooter speed, hood angle, transfer control, LEDs, and telemetry.

Do not try to mention every class or every iteration in the 90-second speech. Mention the strongest systems, leave clear hooks, then use Q&A to show depth.

## 90-Second Speech

Hi judges. For Control, our main goal was to make scoring less dependent on perfect driver timing. The driver still chooses the strategy, but the robot handles the repeatable parts of the cycle: aiming, shooter speed, hood angle, transfer state, LEDs, and telemetry.

The biggest control problem was making the full scoring system agree on where the robot is. We use Pinpoint odometry, corrected by a fixed Limelight seeing AprilTags, so the turret, shooter, and hood all work from the same corrected pose.

For the turret, we went through three versions and ended with a position lookup table that chooses a virtual aim point instead of always aiming at one fixed point. The turret follows that target with motion profiling and PID plus feedforward, so aiming is smoother and more repeatable.

For the shooter, distance selects the flywheel velocity from a LUT. Encoder feedback and voltage-scaled feedforward keep the shooter consistent as the battery changes and after a shot. The hood uses both distance and measured flywheel velocity, so it can adjust when real shooter speed is not exactly the target.

For transfer, three sensors detect when the robot is full, and the state machine controls collect, stop, and shoot. Current is also monitored for protection and debugging.

The main result is that the driver is not manually timing every part of the cycle. Our software turns scoring into a repeatable process, and pages 10 to 12 of our portfolio show the architecture, iteration, and test data behind it.

## Shorter Backup Version

Our Control system is built around driver automation. The driver decides the strategy, while the robot handles the parts that must happen the same way every cycle: corrected pose, aiming, shooter velocity, hood angle, transfer state, LEDs, and telemetry.

For aiming, I would not explain all turret versions in the main speech. I would say: "The turret went through three versions, and the final version uses corrected pose plus a virtual aim-point LUT." That naturally makes judges ask what changed between versions.

For shooting, distance selects flywheel velocity from a LUT. Encoder feedback and voltage-scaled feedforward keep the speed consistent, and the hood LUT uses both distance and measured shooter velocity. The transfer uses three sensors and a state machine so the robot can stop collecting when full and feed when the driver shoots.

The important result is that our driver is not manually timing every part of the cycle. The software turns the scoring cycle into a repeatable process.

## Things To Mention If Asked

### Turret

- V1: Limelight on turret measured target error directly.
- Problem: target could leave camera view while rotating; mechanical stress; still not accurate everywhere.
- V2: Pinpoint pose plus geometry aimed to a fixed field point.
- Problem: odometry drift and one fixed target point still caused misses.
- V3: fixed Limelight corrects odometry, then position LUT creates a virtual aim point.
- `PositionAimLut` uses nearest samples with inverse-square weighting.
- Red and blue samples are mirrored, so both alliances use the same idea.
- `Turret.java` clips target angles to safe limits.
- Turret uses a profiled controller, PIDFF, encoder feedback, voltage sampling, and telemetry snapshots.
- Homing/reset can use current spike detection against the hard stop.

Simple judge answer:

"We did not add complexity for the sake of complexity. Each turret version fixed one failure from the previous version."

### Shooter

- Shooter is velocity-controlled, not fixed power.
- Main motor encoder measures flywheel velocity.
- Follower motor copies the controlled motor power.
- Feedforward predicts needed power; feedback corrects remaining error.
- Feedforward is scaled by battery voltage.
- Shooter ready means measured velocity is within 70 ticks/s of target.
- Code filters flywheel velocity and can detect a shot from a sudden velocity drop.
- Portfolio data: closed loop at 2000 ticks/s had about 21.5 ticks/s average error; open loop had about 168.2 ticks/s.

Simple judge answer:

"Open loop was too dependent on battery and load. Velocity control lets the robot recover after a shot and stay repeatable."

### Hood

- Hood angle is a servo position.
- Hood LUT uses distance and measured flywheel velocity.
- Inputs are normalized before interpolation so velocity units do not overpower distance units.
- Output is clipped between safe servo limits.
- Driver trim exists for calibration.

Simple judge answer:

"Distance alone was not enough. The same distance can need a slightly different hood angle when real shooter speed changes."

### Transfer

- Three sensors track artifact positions.
- Full detection stops collection through the transfer state machine.
- States are `STOP`, `COLLECT`, and `SHOOT`.
- Intake current is monitored and exposed, useful for jam protection and debugging.
- In the current code, the full-sensor stop is active; overcurrent stopping is present as a concept but the stop condition is commented out in `Transfer.java`.

Simple judge answer:

"The transfer is not just motors turning on. It is a small state machine, so the same driver command always produces the same mechanism behavior."

### Localization

- Pinpoint odometry gives fast robot pose.
- Fixed Limelight sees AprilTags without rotating with the turret.
- Vision is used to correct accumulated odometry drift.
- Bad vision frames should be rejected before changing pose.
- Corrected pose is shared by turret, shooter, and hood calculations.

Simple judge answer:

"The Limelight does not directly aim the turret anymore. It improves the robot pose, and then all scoring calculations benefit from that pose."

### Telemetry And Debugging

- Panels telemetry is used for live tuning and debugging.
- Telemetry has competition/debug modes so important data stays visible without flooding the loop.
- Turret and shooter publish snapshots with target, measured value, power, current, and status.
- Loop timing and throttled telemetry matter because control code must run consistently.

Simple judge answer:

"Telemetry is part of the control system for us. It lets us test with data instead of guessing."

### Custom Motor Package

- `MetaMotor`, `VelocityControlledMotor`, and `ProfiledPositionMotor` wrap common motor behavior.
- Motor constants, limits, units, PIDFF coefficients, and motion profiling are centralized.
- Unit conversion classes prevent mixing ticks, radians, and degrees carelessly.
- Motion profile controllers generate smoother references instead of commanding sudden jumps.

Simple judge answer:

"The motor package made the code safer to reuse. Instead of rewriting PID and limits in every subsystem, we built shared motor control tools."

### Autonomous

- Multiple autonomous routines exist for close/far and both alliances.
- Pedro Pathing handles paths and localization.
- Auto choices support alliance flexibility.
- Old auto versions show iteration, but the speech should focus on current reliable routines.

Simple judge answer:

"We built multiple autos because alliance needs change. Control is not only about one perfect route; it is also about having reliable options."

## What Not To Say In The First 90 Seconds

Avoid opening with:

- "inverse distance weighting"
- "normalized feature scaling"
- "low-pass filtered derivative"
- "unit scale bias"
- "PIDFF with trapezoidal references"

Those are good Q&A answers, but they sound memorized if they are the first thing judges hear. Start with the problem and result. Use the technical terms after the judge asks how it works.

## Better Phrasing

Instead of:

"We use normalized feature scaling before interpolation."

Say:

"The hood depends on two different things: distance and real shooter speed. Since those numbers use different units, we scale them before comparing samples."

Instead of:

"We use debounced sensor fusion."

Say:

"The robot waits until the sensors agree long enough before calling the transfer full."

Instead of:

"We use dynamic vector lead compensation."

Say:

"We tested shoot-on-the-move compensation, but it was not reliable enough for matches, so we kept it as a development path instead of claiming it as a match feature."

Instead of:

"Our architecture uses autonomous driver augmentation."

Say:

"The driver chooses the strategy. The robot handles the repeatable timing."

## Judge Baits That Sound Natural

Use these lines only if there is time:

- "The turret went through three versions before we reached the current LUT approach."
- "The shooter data showed closed-loop control was much more repeatable than fixed power."
- "The Limelight improves pose instead of directly aiming the turret."
- "We still keep fallback controls, because a match robot needs recovery options."
- "Some ideas, like shoot-on-the-move, were tested but not used in matches because they were not consistent enough."

These invite good questions without sounding like clickbait.

## Fast Memorization Plan

### The 5-Block Memory Map

Memorize only these blocks, not every exact word:

1. Goal: reduce driver timing.
2. Turret: V1 camera -> V2 pose -> V3 corrected pose plus LUT.
3. Shooter/Hood: distance and velocity LUTs, encoder feedback, voltage compensation.
4. Transfer: sensors plus state machine.
5. Result: faster cycles, fewer setup mistakes, data-backed control.

If you forget a sentence, jump to the next block.

### 20-Minute Drill

1. Read the speech once while looking at it.
2. Cover it and explain only the 5 blocks.
3. Record yourself once.
4. Listen only for places where you sound like you are reading.
5. Rewrite those sentences in your own words.
6. Repeat twice.

### Page Anchor Method

Use the portfolio as memory anchors:

- Page 10: driver automation and control flow.
- Page 11: turret versions and controller table.
- Page 12: data and testing.

If a judge interrupts, point to the right page and continue from the page anchor.

### Practice Questions

Answer each in 20 seconds:

- What problem did software solve for the driver?
- Why did the turret need three versions?
- Why use a virtual aim point?
- Why not just set shooter power?
- Why does the hood use measured velocity?
- What sensors does transfer use?
- What did testing prove?
- What did you try that did not make it into matches?

### Anti-AI Rule

Before judging, replace any sentence you would not naturally say to a teammate in the pit.

Good sentence:

"We tried the camera on the turret first, but it kept creating practical problems."

Bad sentence:

"Our vision subsystem utilized a robust spatial correction pipeline."

## Final Reminders

- Say "we tested" and "we changed" often.
- Mention one limitation honestly. It makes the rest sound more real.
- Do not claim overcurrent stopping in transfer as active unless you re-enable it.
- Do not claim shoot-on-the-move as a current match feature.
- If they ask for code, show `Turret.java`, `PositionAimLut.java`, `Shooter.java`, `HoodAngleLut.java`, `ShooterHoodLuts.java`, and `Transfer.java`.
- End with impact, not algorithms.
