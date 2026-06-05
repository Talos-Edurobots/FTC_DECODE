# Programming page structure

## Page 10 - Driver Automation Architecture

Goal: explain how code reduced driver workload during a full scoring cycle.

Use this page to show that the robot is not only easier to code, but easier to drive under pressure.

### Main message

Our code turns repeated driver actions into automatic subsystem behavior:

```text
driver intent -> subsystem automation -> sensor feedback -> LEDs / ready state
```

### Include

| Section           | Content                                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| Driver automation | turret auto-aligns, shooter/hood presets change together, intake stops when full, LEDs show ready/full state |
| Code structure    | `MainTeleOp`, subsystems, motor package, telemetry                                                           |
| Cycle comparison  | Nationals robot vs current robot button presses                                                              |
| Why it matters    | fewer driver decisions, faster cycles, less timing error, easier driving under pressure                      |

### Driver automation table

| Driver problem             | Automation                                                  |
| -------------------------- | ----------------------------------------------------------- |
| Aim while driving          | turret aims to the goal using robot pose / Limelight mode   |
| Choose shot setup          | one preset changes shooter velocity and hood angle together |
| Know when shooter is ready | LEDs show shooter busy / ready state                        |
| Avoid overfilling          | intake/transfer stops when sensors detect full robot        |
| Feed artifacts             | current transfer system handles the cycle as one action     |

### Small text block

```text
We focused on reducing the number of decisions the driver must make during a cycle. The driver chooses the strategy, but the robot handles repeated timing-sensitive actions: aiming the turret, changing shooter/hood presets, stopping intake when full, and using LEDs to show when the robot is ready.
```

### Cycle comparison

| Robot version | Driver cycle actions |
| --- | --- |
| Nationals - March 15 | press right flicker, release, press left flicker, release, open intake, wait, observe which flicker got artifact, press matching flicker, release |
| Current robot | press one action to collect/feed cycle |

Claim:

```text
The redesign changed the cycle from a sequence of manual timing decisions into a mostly automated scoring pipeline. This reduced driver workload and made cycles more repeatable.
```

### Visuals

- One current cycle flow diagram.
- One before/after button-count comparison.
- LED state icons/colors if there is space.
- No code screenshots unless absolutely needed.

### Avoid

- Listing every class.
- Long explanations of Java structure.
- Saying "we used PID" without saying what problem it solved.

## Page 11 - Feedback Control Systems

Goal: this is the main Control Award page.

This page must show feedback -> algorithm -> match effect.

### Main table

| System    | Feedback                           | Control method            | Match effect                                             |
| --------- | ---------------------------------- | ------------------------- | -------------------------------------------------------- |
| Shooter   | encoder velocity + battery voltage | PIDFF velocity control    | faster spin-up, low overshoot, fast recovery after shots |
| Turret    | robot pose + turret encoder        | geometry + profiled PIDFF | aligns itself without rotating the whole robot           |
| Transfer  | 3 distance sensors + current alert | state machine             | stops intake when full / jam risk                        |
| Telemetry | snapshots + loop timing            | competition/debug modes   | tuning data without flooding drivers                     |

### Shooter block

Use the current data:

| Test | Target | Result |
| --- | ---: | --- |
| closed-loop high speed | 2000 ticks/s | avg error `21.5`, max error `120` |
| open-loop high speed | 2000 ticks/s | avg error `168.2`, max error `240` |
| closed-loop low speed | 1300 ticks/s | avg error `22.2`, max error `100` |
| shot recovery | 1300-2000 ticks/s | recovery about `0.08-0.14s` |

Claim:

```text
The shooter uses encoder feedback and voltage-scaled feedforward to reach target speed quickly without large overshoot. After an artifact slows the flywheel, the controller brings velocity back into shooting range before the next shot.
```

### Turret block

Show the evolution:

```text
V1: Limelight on turret -> easy aiming, but FOV loss and gear skipping
V2: pose-based aiming + motion profile -> smoother and faster
V3: position LUT / vision correction prototype -> corrects field-position aiming errors
```

Use a small graph if available:

- target angle vs measured angle
- xref vs measured position
- vref vs measured velocity

### Transfer block

Show this only if you have test results. If not, keep it short:

```text
The transfer is controlled by a state machine. Sensor feedback detects when the robot is full, and current alerts help stop collection before jams become worse.
```

### Avoid

- Overclaiming vision as match-ready if it is still debug/prototype.
- Saying the LUT solved aiming unless you have before/after shot data.

## Page 12 - Measured Performance

Goal: prove the programming claims with graphs and test results.

This page should be mostly numbers and graphs.

### Layout

| Area | What to show |
| --- | --- |
| Shooter acceleration | graph: closed-loop vs open-loop spin-up from rest |
| Shooter recovery | graph: velocity drop after shot and recovery to target |
| Turret profile | graph/table: settle time, final error, overshoot |
| LUT aiming | table: no LUT made/5 vs LUT made/5 from 6 positions |
| Transfer | table: detection success, false positives, false negatives |
| Driver workload | table: Nationals button sequence vs current cycle action |
| Telemetry | loop Hz competition vs debug |

### Shooter graphs already useful

Use these from `obsidian/graphs/`:

- `shooter acceleration/comparison_keStartWithcontroller_vs_keStartWithoutController.png`
- `shooter acceleration/comparison_keHighSpeedWithController_vs_keHighSpeedWithoutController.png`
- `drop/highVelWithControllerDrop_acceleration_threshold.png`
- `drop/lowVelWIthControllerDrop_acceleration_threshold.png`

### Data still missing

| Missing result | Needed for |
| --- | --- |
| turret settle time / final error | prove motion-profiled turret control |
| LUT before/after shot accuracy | prove position LUT improves aiming |
| transfer 20-trial sensor test | prove automation reliability |
| loop Hz competition/debug | prove telemetry design does not hurt match loop |
| vision accepted/rejected corrections | only if claiming vision relocalization |

### Final result wording

Use this format:

```text
Problem -> Feedback -> Control method -> Result -> Evidence
```

Example:

```text
The shooter missed when velocity changed after shots. We used encoder velocity and battery voltage in a PIDFF controller. Compared with open loop, average velocity error at 2000 ticks/s dropped from 168.2 to 21.5 ticks/s, and recovery after shot drops was about 0.08-0.14s.
```

### Avoid

- Big paragraphs.
- Raw code.
- Graphs without a one-line conclusion.
- Claims without numbers.

## Priority if space is tight

1. Shooter velocity control with graphs and numbers.
2. Driver automation / button-count reduction.
3. Turret aiming evolution + motion profile.
4. LUT before/after shot accuracy.
5. Transfer automation.
6. Telemetry architecture.
7. Vision relocalization only as prototype unless fully tested in match code.
