# AI note - Control Award review and action plan

This note was written by AI after reading the Obsidian notes, `doc/`, the control-related Java code, and the simulator / notebook scripts. Treat it as critique and planning material, not final portfolio text.

## Short verdict

The codebase has real Control Award substance, but the portfolio structure should be more honest and more evidence-driven.

The strongest story is not "we used PID." Many FTC teams use PID. The stronger story is:

- the robot uses pose, encoders, sensors, and vision to reduce driver workload
- the turret evolved through several control approaches
- shooter velocity is closed-loop and measured
- transfer/intake has sensor-based automation
- telemetry is designed around cost, mode, and loop performance
- simulator and debug OpModes are used to create/tune LUTs and collect data

The weak point is evidence. The code can produce data, but the data is not yet organized into clean, repeatable experiments that a judge can understand quickly.

## What I would claim confidently

These are supported by code and should be in the portfolio.

### 1. Motion-profiled turret control

Code evidence:

- `TeamCode/.../subsystem/Turret.java`
- `TeamCode/.../motor/facade/ProfiledPositionMotor.java`
- `TeamCode/.../motor/math/controllers/TrapezoidalMotionProfileController.java`

Claim:

The turret does not directly slam motor power based only on position error. It generates a motion profile with reference position, velocity, and acceleration, then follows that reference with PIDFF control.

Why judges care:

- smoother aiming
- less mechanical stress
- faster target changes without gear skipping
- better repeatability than raw power or simple P control

Evidence to collect:

- turret target angle
- measured angle
- reference velocity
- measured velocity
- settle time
- final error

Useful graph:

- target angle vs measured angle
- xref vs measured position
- vref vs measured velocity

### 2. Shooter velocity regulation

Code evidence:

- `TeamCode/.../subsystem/Shooter.java`
- `TeamCode/.../motor/facade/VelocityControlledMotor.java`
- `TeamCode/.../motor/math/controllers/PIDFFVelocityController.java`

Claim:

The shooter uses encoder velocity feedback and feedforward, scaled by battery voltage, to keep the flywheel near target speed.

Why judges care:

- shot distance and consistency depend on flywheel speed
- battery sag and shot load affect speed
- closed-loop velocity control lets the robot wait until the shooter is ready

Evidence to collect:

- target velocity
- measured velocity
- spin-up time
- velocity drop after each shot
- recovery time after shot
- success/failure of shots at different target velocities

Useful graph:

- target velocity vs measured velocity over time
- detected shot drops
- recovery time after each artifact

### 3. Transfer/intake automation

Code evidence:

- `TeamCode/.../subsystem/Transfer.java`
- `TeamCode/.../subsystem/ColorSensors.java`
- `TeamCode/.../subsystem/Intake.java`

Claim:

The transfer/intake is state-based. During collection, sensor detection or intake overcurrent automatically stops collection.

Be precise:

- The code uses `DistanceSensor`, even if the physical device is a color/distance sensor.
- The current code updates the sensors at 10 Hz, not 5 Hz.

Why judges care:

- reduces driver reaction burden
- prevents overfilling
- helps avoid jams/current issues
- shows use of external feedback

Evidence to collect:

- detection accuracy
- false positives
- false negatives
- time from third artifact detection to intake stop
- number of avoided jams/current alerts

### 4. Structured telemetry system

Code evidence:

- `TeamCode/.../telemetry/TelemetryHub.java`
- `TelemetryCollector.java`
- `TelemetryMode.java`
- `TelemetryCostClass.java`
- `ThrottledValue.java`
- `ShooterTelemetrySnapshot.java`
- `TurretTelemetrySnapshot.java`

Claim:

Telemetry is treated as part of the control architecture, not random print statements. Competition telemetry is separated from debug telemetry, and expensive values like continuous current readings are throttled.

Why judges care:

- loop consistency matters for control quality
- telemetry can slow match code
- programmers can tune mechanisms without burying drivers in debug data

Evidence to collect:

- loop Hz in competition telemetry
- loop Hz in debug telemetry
- current reads throttled at 0.1 s or 0.2 s
- example screenshots of competition vs debug telemetry

## What I would not claim yet

These exist partially, but they are not fully integrated or proven enough to describe as finished match systems.

### 1. Do not claim full match vision relocalization yet

The `VisionRelocalizationSubsystem` exists inside `Debugger.java`, not as a real subsystem used by `MainTeleOp` or `NewAuto`.

That means the correct wording is:

"We prototyped and tested vision-based relocalization using Limelight AprilTag data."

Not:

"The robot automatically relocalizes during every match shot."

Action:

- move `VisionRelocalizationSubsystem` into `subsystem/`
- integrate it into `MainTeleOp`
- optionally integrate it into auto
- log accepted/rejected corrections
- prove it improves pose accuracy or shot accuracy

### 2. Do not overclaim the position aiming LUT

The `PositionAimLut` implementation is good, but in `Turret.java`:

- `positionAimLutEnabled` defaults to false
- red has samples
- blue LUT is empty

Correct wording before more testing:

"We implemented a position-based virtual aim-point LUT and are using it as a tunable aiming correction system."

Stronger wording only after testing:

"The LUT improved shot accuracy from X% to Y% across tested field positions."

Action:

- decide whether LUT is competition-enabled
- add blue alliance mirroring or blue samples
- log selected virtual aim point
- run a before/after shot test

### 3. Do not claim shoot-on-the-move as successful

The code has a `lookToGoalWhileMoving` method, but the current model is a rough pose shift using full velocity vector and distance. It is not a strong physics model.

Correct wording:

"We experimented with shoot-on-the-move compensation, but removed it from match strategy because it reduced consistency."

This is actually a good engineering lesson if presented honestly.

## Measurement classes already present

You do have measurement tools. The problem is organization.

### Existing tools

- `KeCharacterizationOpMode`
  - logs shooter velocity vs applied voltage
  - useful for feedforward tuning

- `KaTestOpMode`
  - logs turret velocity, applied voltage, current, power, position, xref, vref, aref, target position, time
  - useful for turret profile tuning graphs

- `TestThroughput`
  - logs shooter velocity, shooter power, intake current, shooter current, target
  - useful for detecting shot events and throughput

- `CollectData`
  - logs shot success/failure with pose, turret angle, shooter velocity, hood angle, velocity
  - this is the most directly useful Control Award data collector

- `ShooterTelemetrySnapshot`
  - target velocity
  - measured velocity
  - filtered velocity
  - applied power
  - hood angle
  - busy/ready
  - impact detected
  - current

- `TurretTelemetrySnapshot`
  - control mode
  - LUT status
  - target angle
  - measured angle
  - aim point
  - reference velocity
  - reference acceleration
  - overcurrent
  - current

## Measurement problem

Most measurement is currently emitted through `Log.d` and then analyzed manually in Colab-style scripts. That works for programming, but it is weak for portfolio evidence unless the process is documented clearly.

Recommended structure:

```text
obsidian/
  control-award/
    experiments/
      shooter_velocity_test.md
      turret_profile_test.md
      lut_aiming_test.md
      transfer_detection_test.md
    results/
      shooter_velocity_summary.md
      turret_profile_summary.md
      lut_accuracy_summary.md
```

Each experiment note should have:

- goal
- robot setup
- code / OpMode used
- variables recorded
- number of trials
- result table
- conclusion
- what changed in the robot because of the test

## Add this to code if possible

### 1. Reusable CSV experiment logger

Add a small logger so all debug OpModes write consistent CSV rows instead of scattered `Log.d` strings.

Suggested class:

```text
TeamCode/.../main/experiments/CsvExperimentLogger.java
```

Suggested API:

```java
logger.header("time,targetVelocity,measuredVelocity,power,hood,success");
logger.row(now, targetVelocity, measuredVelocity, power, hood, success);
```

Even if it still writes to Logcat, the class gives structure and makes the experiment process easier to explain.

### 2. Shot trial logger

Extend `CollectData` or replace it with a cleaner `ShotAccuracyExperiment`.

Record:

- trial number
- timestamp
- alliance
- robot x
- robot y
- robot heading
- robot velocity
- turret target angle
- turret measured angle
- turret error
- virtual aim point x/y
- shooter target velocity
- shooter measured velocity
- hood position
- shot success

This is the most important dataset for winning Control Award.

### 3. Turret settle test

Create a test that commands the turret through fixed angles and logs:

- target angle
- measured angle
- time to within 1 degree
- overshoot
- peak current

Use this to prove motion profiling helped.

### 4. Shooter recovery test

Use the existing shooter impact detection to measure:

- velocity before shot
- velocity drop
- recovery time to within tolerance
- whether next shot was fired too early

This proves the velocity controller has match value.

## Structure recommendation for Obsidian

Current notes are too flat. They should be reorganized around the award story.

Recommended folders:

```text
obsidian/
  Control Award/
    00 Control Award Summary.md
    01 System Architecture.md
    02 Turret Aiming.md
    03 Shooter Velocity.md
    04 Transfer Automation.md
    05 Telemetry and Measurements.md
    06 Experiments and Results.md
    07 Judge Talking Points.md
  Control Award/Experiments/
    Shooter Velocity Test.md
    Turret Motion Profile Test.md
    Position LUT Shot Accuracy Test.md
    Transfer Sensor Reliability Test.md
    Vision Relocalization Test.md
```

## Suggested portfolio page structure

Use one polished Control Award page, not many disconnected paragraphs.

### Section 1 - Control system overview

Small table:

| System                | Feedback                   | Control method            | Match impact                         |
| --------------------- | -------------------------- | ------------------------- | ------------------------------------ |
| Turret                | pose + encoder             | geometry + profiled PIDFF | automatic aiming                     |
| Shooter               | encoder velocity + voltage | PIDFF velocity loop       | consistent shots                     |
| Transfer              | 3 sensors + current alert  | state machine             | fewer jams                           |
| Telemetry             | structured snapshots       | modes + throttling        | better loop consistency              |
| Vision relocalization | AprilTag pose              | filtered correction       | prototype / future unless integrated |

### Section 2 - Main innovation

Make turret + shooter the main story.

Use this chain:

```text
Pinpoint pose -> aim geometry / LUT -> turret motion profile -> shooter velocity ready -> transfer feeds artifact
```

### Section 3 - Engineering iteration

Use V1 / V2 / V3, but be honest.

V1:

- Limelight-on-turret aiming
- easy to implement
- failed due to FOV, mechanical stress, and aiming at tag center

V2:

- pose-based turret aiming with motion profile
- improved speed and removed FOV problem
- failed because odometry drift and single target point were not enough

V3:

- position-based virtual aim LUT
- vision relocalization prototype
- telemetry-driven tuning
- SOTM removed because it hurt consistency

### Section 4 - Evidence

Do not write this without numbers.

Minimum numbers to collect:

- shooter spin-up time
- shooter average velocity error
- shooter recovery time after shot
- turret average settle time
- turret final angle error
- LUT shot accuracy before/after
- transfer full detection accuracy
- loop Hz in competition vs debug telemetry

## Things to fix in the existing notes

- `Shooter.md` is unfinished. Finish it.
- `Intake and Transfer.md` says "states the turret should be." That should be transfer/intake.
- `Turret.md` says "Limelight doesn't create errors." That is false. Say it has useful absolute feedback but also latency, FOV, calibration, lighting, and bad-frame risks.
- `Trapezoidal Motion Profiling.md` formula should not use `signum(kv)` for static feedforward. It should use direction of motion/reference, such as `sign(v_ref)`, or fallback to position error when reference velocity is zero.
- `Simulator.md` needs more than one sentence. Explain coordinate system, controls, LUT export, and how samples move into robot code.
- `Turret LUT.md` should say it maps robot field position to virtual aim point, not just "position-target aiming position pairs."

## Judge-facing wording to avoid

Avoid:

- "The Limelight doesn't create errors."
- "The LUT solved the problem."
- "The turret is consistent and accurate."
- "SOTM works" if you removed it.
- "Sensor fusion" unless you can explain exactly what is fused and where in code.

Use:

- "We measured..."
- "We rejected..."
- "We disabled..."
- "This improved X by Y..."
- "The current limitation is..."
- "The fallback is..."

Judges trust teams that can explain tradeoffs and failures.

## Most important next action

Run the shot accuracy experiment.

Suggested test:

1. Pick 6 field positions.
2. At each position, shoot 5 artifacts using normal center-goal aiming.
3. Record successes.
4. Enable/adjust LUT.
5. Shoot 5 more artifacts from each position.
6. Record successes.
7. Make one table and one field diagram.

That one experiment can turn the portfolio from "we wrote advanced code" into "we engineered a measured improvement."

## Final recommendation

Write the portfolio yourself, but base it on measured claims:

- what feedback the robot uses
- what control algorithm uses that feedback
- what problem it solved
- what evidence proves it helped
- what failed and what you changed

The current codebase can support a strong Control Award submission. The missing work is not another fancy algorithm. The missing work is disciplined proof.
