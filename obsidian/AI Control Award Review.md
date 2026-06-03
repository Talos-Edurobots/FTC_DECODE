# Control Award documentation review

This is a blunt review of the Obsidian vault and the robot data under `data/`. It is not final portfolio text. It is a correction list and evidence plan.

## Short verdict

The robot has real Control Award material. The documentation does not yet prove it well enough.

The strongest systems are:

- shooter velocity control with encoder feedback and voltage-scaled feedforward
- pose-based turret aiming with a profiled position controller
- transfer/intake automation using three distance sensors and overcurrent protection
- telemetry snapshots with competition/debug modes and throttled expensive values
- simulator/LUT tooling for aiming calibration

The weak part is not the algorithms. The weak part is proof. The vault still uses too many claims like "consistent", "accurate", and "solved" without numbers attached. Judges do not need more buzzwords. They need measured before/after behavior, failure cases, and match relevance.

## Biggest problems

### 1. The shooter evidence is strong, but the notes do not use it

The shooter is currently the best evidence story in the repo.

From the throughput drop data:

| Dataset                                          | Controller | Target | Average absolute velocity error | Within 70 ticks/s |
| ------------------------------------------------ | ---------- | -----: | ------------------------------: | ----------------: |
| `data/throuput/highVelWithControllerDrop.csv`    | yes        |   2000 |              about 21.5 ticks/s |             94.9% |
| `data/throuput/highVelWithoutControllerDrop.csv` | no         |   2000 |             about 168.2 ticks/s |              6.8% |
| `data/throuput/lowVelWIthControllerDrop.csv`     | yes        |   1300 |              about 22.2 ticks/s |             92.5% |

This is the kind of evidence that wins arguments. The shooter note should lead with this instead of only explaining PIDFF.

What to say:

> Closed-loop shooter control held the flywheel within our 70 ticks/s ready tolerance for about 95% of high-speed samples, compared with about 7% for open-loop high-speed control.

What not to say:

> We used PID and it made the shooter accurate.

That is too generic.

### 2. Turret claims are ahead of turret evidence

The turret system is impressive, but the data is not complete enough for the claims in the vault.

The turret KA CSVs include:

- power
- acceleration reference
- velocity reference
- position reference
- measured velocity
- measured position
- current
- target positions

But they do not include time. Without time, you cannot honestly calculate settle time, response time, or recovery behavior from those files. You can show that the profile exists. You cannot prove "fast lock" or "settled in X seconds" from the current CSVs.

Fix the logger before running more turret experiments. Add:

- `time`
- `trial_id`
- `target_deg`
- `measured_deg`
- `error_deg`
- `within_tolerance`
- `peak_current`

Minimum useful turret claim:

> The turret uses a trapezoidal reference and follows position, velocity, and acceleration targets instead of directly slamming power from position error.

Stronger claim only after new data:

> The turret settled within 1 degree in X seconds with Y degrees overshoot and Z amps peak current.

### 3. The LUT is promising, not proven

The code has `PositionAimLut`, and red-side samples exist. Blue-side samples are empty in `Turret.java`.

That means the vault should not say the LUT "solved" aiming. It should say:

> We implemented a position-based virtual aim-point LUT for red-side calibration and are testing it as an aiming correction layer.

The missing experiment is obvious:

1. Pick 6 field positions.
2. Shoot 5 artifacts with normal goal-center aiming.
3. Shoot 5 artifacts with LUT aiming.
4. Record made/missed, robot pose, heading, turret target, turret measured angle, shooter velocity, and hood angle.
5. Keep the bad results. Do not retune halfway through the before/after set.

Until that table exists, the LUT is an implementation, not a proven improvement.

### 4. Vision relocalization is not a match feature yet

The relocalization code is inside `Debugger.java`, not a normal subsystem used by match TeleOp or autonomous. `MainTeleOp` also defaults Limelight aiming off.

Correct wording:

> We prototyped Limelight AprilTag relocalization and tested accepted/rejected pose corrections.

Incorrect wording:

> The robot uses vision relocalization during every match.

To make the stronger claim, move the relocalization logic into `subsystem/`, integrate it into `MainTeleOp` or autonomous, and log:

- Pinpoint pose before correction
- Limelight pose
- correction accepted/rejected
- rejection reason
- pose error before/after at taped field positions

### 5. Shoot-on-the-move should be presented as a rejected design

The vault says SOTM was tested and removed. That is good. Keep it honest.

The TeleOp call to `lookToGoalWhileMoving` is commented out. The current method uses a rough pose shift based on full robot velocity and distance. That is not a real projectile-flight-time model.

Use this as an engineering lesson:

> We tested shoot-on-the-move compensation, but it reduced consistency and slowed cycles, so we removed it from match strategy.

Do not imply it is a working feature.

### 6. The data folder is useful but messy

Current problems:

- `data/throuput` is misspelled.
- `lowVelWithoutControllerDrop.csv` is empty.
- `lowVelWIthControllerDrop.csv` has inconsistent capitalization.
- The acceleration datasets were moved to `data/shooter acceleration`, while related drop datasets are still under `data/throuput`.
- Several files start mid-run instead of at a clean test start.
- CSVs have no date, battery label, OpMode, trial number, robot setup, or notes.

For programming, this is survivable. For award evidence, it is weak.

Recommended structure:

```text
data/control-award/
  shooter_velocity/
    2026-06-03_high_closed_loop_drop.csv
    2026-06-03_high_open_loop_drop.csv
    summary.md
  turret_profile/
    2026-06-03_0_to_60_trial_01.csv
    summary.md
  lut_accuracy/
    2026-06-03_before_after_lut.csv
    summary.md
  transfer_detection/
    2026-06-03_detection_trials.csv
    summary.md
```

Every CSV should include enough metadata to be understandable without asking the programmer who ran the test.

### 7. The scripts are not reproducible evidence tools

The scripts in `obsidian/scripts` look like exported Colab notebooks. They hardcode `/content` paths, use `display()`, mount Google Drive, and expect filenames that do not match the current repo.

That is fine for rough exploration. It is not good documentation.

Rewrite them as local scripts that accept command-line paths:

```text
python obsidian/scripts/analyze_shooter_velocity.py data/control-award/shooter_velocity/file.csv
python obsidian/scripts/analyze_turret_profile.py data/control-award/turret_profile/file.csv
```

Each script should save:

- one graph image
- one small markdown summary
- the computed numbers used in the portfolio

## Broken or weak notes

### `obsidian/subsystem/Shooter.md`

This note is unfinished and has an unclosed code fence. It should be finished before exporting anything.

Add:

- the measured closed-loop vs open-loop table
- the `isBusy()` tolerance
- what happens before feeding
- recovery/drop detection explanation
- one graph from the throughput data

### `obsidian/New review.md`

This looks like an accidental duplicate of the shooter note and also has an unclosed code fence. Delete it or replace it with a real top-level review. Do not keep duplicate half-notes.

### `obsidian/subsystem/Turret.md`

This note has the right iteration structure, but the wording is too loose.

Fix these:

- "The limelight doesn't create errors" is false.
- "Consistent and accurate aiming" needs numbers or should be softened.
- "sensor fusion" must be explained exactly or removed.
- "LUT solves the aiming issues" should become "LUT is being tested to correct position-dependent aiming error."

Better structure:

- V1: Limelight-on-turret aiming, failed because of FOV, target choice, and mechanical stress.
- V2: pose-based aiming with motion profiling, improved mechanics but still suffered from odometry drift and single target-point aiming.
- V3: fixed Limelight relocalization prototype plus position-based virtual aim LUT, promising but needs before/after shot data.

### `obsidian/subsystem/Intake and Transfer.md`

This note is too short and has sloppy wording.

Fix:

- "states the turret should be" should be "states the transfer should be in."
- "allerts" should be "alerts."
- mention the real code behavior: three `DistanceSensor` readings, 10 Hz update rate, 0.4 s full debounce, stop on full or intake overcurrent.

### `obsidian/Simulator.md`

One sentence is not enough. This is a major part of the LUT story.

Add:

- coordinate system
- how robot pose is selected
- how virtual aim points are chosen
- how samples are exported
- how samples move into `PositionAimLut`
- limitations of simulator tuning versus live shot testing

### `obsidian/LUTS/Turret LUT.md`

This note needs to explain that the LUT maps robot field position to a virtual aim point. It should also admit that live shot accuracy is the real validation, not the simulator screenshot.

### `obsidian/Main structure.md`

This is basically empty. Either fill it with the architecture map or delete it.

## What to claim confidently

### Shooter velocity control

Supported by code and data.

Claim:

> The shooter uses encoder velocity feedback, PIDFF, and voltage-scaled feedforward. In high-speed testing, closed-loop control held velocity near target far better than open-loop feedforward alone.

Evidence:

- target vs measured velocity graph
- closed-loop vs open-loop error table
- ready tolerance percentage

### Transfer automation

Supported by code, but needs trial data.

Claim:

> During collection, three distance sensors detect fullness and the intake stops automatically when the robot is full or overcurrent is detected.

Evidence still needed:

- 20 collection trials
- false positives
- false negatives
- stop response time
- jam/current events

### Motion-profiled turret

Supported by code, partially supported by data.

Claim:

> The turret follows a trapezoidal motion profile with PIDFF instead of direct raw position control.

Evidence still needed:

- timed settle tests
- final angle error
- overshoot
- peak current

### Telemetry architecture

Supported by code, needs loop-rate evidence.

Claim:

> Competition telemetry is separated from debug telemetry, and expensive readings are throttled to protect loop consistency.

Evidence still needed:

- competition loop Hz
- debug loop Hz
- screenshot/example of each mode

## Judge-facing language to avoid

Avoid:

- "The Limelight doesn't create errors."
- "The LUT solved the problem."
- "The turret is consistent and accurate."
- "SOTM works."
- "Sensor fusion" unless you can point to the exact code and explain the filter.
- "It worked."

Use:

- "We measured..."
- "We rejected..."
- "We disabled..."
- "This improved X by Y..."
- "The limitation is..."
- "The fallback is..."

## Priority action list

1. Finish `Shooter.md` with the actual closed-loop vs open-loop numbers.
2. Fix or delete `New review.md`.
3. Rewrite `Turret.md` so it stops overclaiming.
4. Add time and trial metadata to turret logs.
5. Run the LUT before/after shot accuracy test.
6. Rename and reorganize the data folder for Control Award evidence.
7. Rewrite analysis scripts so they run locally from repo data.
8. Add transfer detection trial results.
9. Add telemetry loop-rate results.
10. Only then write final judge-facing portfolio text.

## Final take

The codebase is more serious than the vault makes it look. Right now the documentation undersells the shooter, overclaims the turret/LUT/vision side, and leaves too much evidence trapped in messy CSVs and Colab-style scripts.

Be ruthless with the wording. If there is no number, call it a prototype, a design decision, or a lesson learned. If there is a number, put it in a table and make it the center of the story.
