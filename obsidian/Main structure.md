# Programming portfolio - final three-page content

This is the content that should appear in the portfolio, not an outline of the writing process. Keep each page visual and use short captions instead of paragraphs.

## Page 1 - Driver automation

### Main statement

Our software lets the driver choose when to collect and shoot while the robot handles localization correction, field-centric driving, turret aiming, shooter velocity, hood position, full-capacity detection, and status feedback.

### Control flow

```text
Pinpoint odometry + fixed Limelight AprilTag observations
   `-> validated pose relocalization -> corrected robot pose

corrected robot pose
   |-> turret position LUT -> virtual aim point -> profiled turret controller
   |-> distance LUT -> shooter velocity -> PIDFF flywheel controller
   `-> distance + measured velocity LUT -> hood position

3 distance sensors + intake current -> transfer state machine
shooter / turret / transfer state -> LEDs + Panels telemetry
```

### Driver workload table

| Driver intent   | Robot response                                                                                            |
| --------------- | --------------------------------------------------------------------------------------------------------- |
| Drive           | Field-centric drive with acceleration limiting; stick click enables 25% slow mode                         |
| Collect         | Transfer enters `COLLECT`; intake stops automatically when full or overcurrent                            |
| Shoot           | Holding the bumper opens the gate and runs the transfer                                                   |
| Localize        | A fixed Limelight above the intake uses AprilTags to correct accumulated odometry drift                   |
| Aim             | Turret continuously calculates its angle from corrected robot pose and the interpolated virtual aim point |
| Select fallback | Driver can use a face-forward override or fixed close/far values if the LUT system is disabled            |
| Read status     | LEDs show shooter/target and transfer states; telemetry exposes detailed debug values                     |

### Iteration evidence

| Earlier robot                                                | Current robot                                                                                        |
| ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------- |
| Driver timed multiple flickers and watched artifact position | One transfer state machine controls collection, stopping, and feeding                                |
| Two fixed velocity/hood combinations                         | Velocity and hood change continuously from pose and measured shooter velocity                        |
| Turret camera could lose the target while rotating           | Fixed Limelight relocalizes the pinpoint odometry computer while pose/LUT aiming controls the turret |
| Turret aimed at one fixed field target                       | Position LUT changes the virtual aim point across the field                                          |

### Visuals

- One current scoring-cycle diagram.
- One before/after driver-action comparison.
- Small LED legend.

## Page 2 - Adaptive scoring control

### Main statement

The strongest technical page should show how the scoring system evolved into one corrected-pose control pipeline, not just list every subsystem.

| System       | Input / feedback                                             | Algorithm                                                     | Output                   |
| ------------ | ------------------------------------------------------------ | ------------------------------------------------------------- | ------------------------ |
| Localization | Pinpoint odometry + fixed Limelight AprilTag pose            | Observation validation + pose correction                      | Corrected field pose     |
| Turret       | Corrected field pose + turret encoder                        | 2D inverse-distance LUT, geometry, trapezoidal profile, PIDFF | Turret angle             |
| Shooter      | Corrected goal distance + flywheel encoder + battery voltage | Linear distance LUT + velocity PIDFF                          | Flywheel velocity        |
| Hood         | Corrected goal distance + measured flywheel velocity         | 2D inverse-distance LUT                                       | Hood servo position      |
| Transfer     | Three distance sensors + intake current                      | `STOP`, `COLLECT`, `SHOOT` state machine                      | Intake and gate commands |

### Portfolio text

The Limelight is fixed above the intake instead of rotating with the turret. It observes field AprilTags and corrects accumulated Pinpoint odometry drift. Invalid or implausible observations are rejected before correction, so one poor frame cannot abruptly move the robot pose. The corrected pose is shared by the turret, shooter, and hood calculations.

Previously, two fixed velocity-angle pairs only worked in limited regions. `MainTeleOp` calculates corrected distance to the goal every loop. A linear LUT sets flywheel velocity, while a second LUT selects hood position using both distance and measured flywheel velocity. This lets the hood compensate when real velocity differs from the requested velocity.

The turret LUT stores calibrated pairs of robot position and virtual aim point. The three nearest samples are weighted by inverse squared distance, then geometry converts the interpolated aim point into a turret angle. A trapezoidal motion profile and PIDFF controller move the turret without abrupt acceleration.

The driver can trim the hood during testing or use fixed close/far values if the LUT system is disabled. A face-forward turret override remains available, while corrected-pose/LUT aiming is the default.

### What should actually be on Page 2

Use three compact blocks instead of one long paragraph.

#### Block A - Turret evolution

| Version | Main idea                                                                       | Why it changed                                                                                                      |
| ------- | ------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| V1      | Limelight mounted on the turret directly measured target error                  | Lost target with turret rotation, stressed mechanics, and still aimed poorly from some field positions              |
| V2      | Pinpoint pose plus trapezoidal motion profiling aimed to one fixed field target | Removed camera FOV dependence and mechanical stress, but odometry drift and one fixed target still limited accuracy |
| V3      | Fixed Limelight relocalization plus position aim LUT                            | Corrected odometry drift and changed the virtual aim point across the field                                         |

Judge message: the turret design did not just become more complicated. Each version removed a specific control failure from the previous one.

#### Block B - Shooter controller

Keep one short formula callout:

$$
power = PID(v_{target}-v_{measured}) +
{k_s\,sign(v_{target}) + k_v v_{target} \over V_{battery}}
$$

One sentence under it:

```text
Encoder feedback corrects velocity error, while voltage-scaled feedforward predicts the power needed to hold speed before error grows.
```

#### Block C - Tuning and evidence

Use one caption-sized note for tuning:

```text
We tuned feedforward constants from logged voltage-versus-velocity data using `ke_kv_calculator.py`, then verified the result with closed-loop error and recovery graphs.
```

This belongs on the page because it proves engineering process, but it should stay small. Judges will care more about the measured improvement than the script name.

### Shooter result

| Test                      | Average absolute error | Within 70 ticks/s |
| ------------------------- | ---------------------: | ----------------: |
| Closed loop, 2000 ticks/s |     about 21.5 ticks/s |             94.9% |
| Open loop, 2000 ticks/s   |    about 168.2 ticks/s |              6.8% |
| Closed loop, 1300 ticks/s |     about 22.2 ticks/s |             92.5% |

Caption: Encoder feedback and voltage-scaled feedforward made flywheel speed substantially more repeatable than open-loop control.

### Visuals

- Adaptive scoring pipeline diagram.
- Turret V1 -> V2 -> V3 evolution strip.
- One shooter closed-loop/open-loop graph.
- One small formula callout box.
- Small localization diagram showing Pinpoint prediction and Limelight correction.

## Page 3 - Validation and engineering process

### Main statement

We validate controls by measuring their match effect, not only whether the code follows a target.

### LUT validation: scoreable field coverage

The LUT improvement is best represented as an area, because aiming success at a field point is effectively pass/fail: a correctly calibrated shot enters the goal, while an incorrect angle or trajectory misses.

1. Define a grid of reachable robot poses and test them in the simulator.
2. Test the previous system using its two fixed velocity/hood pairs and fixed turret target.
3. Mark each pose as scoreable or not scoreable.
4. Repeat with turret, shooter velocity, and hood LUTs enabled.
5. Overlay the two maps and calculate scoreable area or scoreable grid-cell count.
6. Physically spot-check boundary and representative poses before claiming match reliability.

| Configuration | What the map should show                                                                  |
| ------------- | ----------------------------------------------------------------------------------------- |
| Before LUTs   | Limited scoring regions around the two calibrated shot combinations                       |
| After LUTs    | Larger continuous scoring region from interpolated velocity, hood, and virtual turret aim |

Use two field heatmaps with the same scale. Label the added scoreable area and include a few boundary failures rather than hiding them.

### Other evidence

| Evidence                                 | Portfolio conclusion                                                                    |
| ---------------------------------------- | --------------------------------------------------------------------------------------- |
| Shooter acceleration and recovery graphs | Closed-loop control reaches and recovers target velocity consistently                   |
| Turret target vs measured angle          | Profiled control follows a smooth reference within the chosen tolerance                 |
| 20 transfer trials                       | Full detection and overcurrent protection stop collection reliably                      |
| Competition vs debug loop rate           | Tiered and throttled telemetry preserves loop performance                               |
| Known-position relocalization test       | Valid AprilTag observations reduce pose error without accepting large false corrections |
| Driver-action comparison                 | Automation reduces repeated timing decisions                                            |

### Development tools

Include one compact line or icon row, not a separate section:

```text
Android Studio: code and debugging | Panels: live tuning and graphs |
Google Colab/Python: data analysis | Pedro Pathing: localization and path following |
Custom simulator: LUT calibration and field-coverage validation
```

These tools belong in the portfolio only when connected to the engineering process. The important story is what each tool enabled, not a list of software logos.

### Evidence still needed

- Fix or verify alliance-specific goal distance for the shooter and hood LUTs.
- Record pose error before and after Limelight relocalization at known field points.
- Report accepted/rejected vision observations and rejection reasons.
- Generate before/after scoreable-area maps and report the area or grid-cell increase.
- Spot-check simulator predictions at representative and boundary field poses.
- Add timed turret settle/error data if a numerical turret-performance claim is used.
- Run transfer reliability trials.
- Record competition/debug loop-rate results.

## Space priority

1. Corrected-pose adaptive scoring pipeline.
2. Turret V1 -> V2 -> V3 evolution.
3. Limelight relocalization evidence.
4. Shooter closed-loop evidence.
5. Before/after scoreable field coverage.
6. Driver and transfer automation.
7. Turret profile and telemetry evidence.
8. Development tools as one small footer row.
