# FTC Portfolio and Control Award Research

Research date: 2026-05-20

## User and Team Context

- The user needs to write the portfolio/notebook material for the FTC Control Award.
- The team is also planning a larger engineering notebook containing details about every department.
- The user's department is programming, so the notebook material should explain the software architecture, autonomous routines, control loops, sensors, telemetry, and lessons learned.
- The active codebase is FTC_DECODE, a Java FTC SDK project using Pedro Pathing, goBILDA Pinpoint localization, Limelight, custom motor-control facades, mechanism subsystems, and a structured telemetry system.
- The requested source to review was FTCPortfolioLab: https://www.ftcportfoliolab.org/portfolio

## Sources Consulted

- FIRST Tech Challenge 2025-2026 Competition Manual, Section 6 Awards: https://ftc-resources.firstinspires.org/file/ftc/game/manual
- FTCPortfolioLab portfolio library: https://www.ftcportfoliolab.org/portfolio
- FTCPortfolioLab home/overview: https://www.ftcportfoliolab.org/
- Game Manual 0 awards guide: https://gm0.org/en/latest/docs/awards/index.html
- Project Robotica engineering portfolio guide: https://projectrobotica.wiki/wiki/FTC%3AEngineering_Portfolio
- FIRST Tech Challenge Docs home: https://ftc-docs.firstinspires.org/

## Current FTC Portfolio Rules

The current official source is the 2025-2026 FIRST Tech Challenge Competition Manual. Important rules and judging implications:

- Teams may submit a portfolio for judged awards, and judges use the portfolio, structured interview, pit interviews, and event observations as judging inputs.
- The portfolio must have one cover page and no more than 15 pages of content.
- The cover page must identify the team number and may include team name, table of contents, organizations, sponsors, logo, motto, and team/robot picture.
- The portfolio must use US Letter or A4 pages.
- If submitted digitally, the complete file must be under 15 MB.
- The portfolio should only include progress, challenges, and accomplishments since January 1, 2025.
- Judges will not use material beyond the 15 content pages.
- Readability matters. The manual warns that very small fonts or low-contrast design choices may make content unusable for judges.
- Judges will not open linked documents, websites, or videos from the portfolio during deliberation. Links can still be useful for team use or pit discussion, but core evidence must be inside the portfolio itself.
- AI or writing aids may be used, but must be credited if used in the submitted portfolio.

Practical implication: the portfolio should be a concise, judged-award-facing summary. It should not be a code dump or a complete engineering notebook. Use diagrams, tables, and short explanations that answer award criteria directly.

## Current Control Award Criteria

The 2025-2026 manual defines the Control Award as recognition for using sensors and software to increase robot functionality during gameplay. It explicitly includes autonomous operation, intelligent mechanical control, and sensor-based improvement. Control solutions may be used in AUTO and/or TELEOP.

Required criteria:

- The team must submit a portfolio.
- The portfolio must include hardware and/or software control components on the robot.
- The portfolio must explain which challenge each component or system solves.
- The portfolio must explain how each component or system works.
- The team must use one or more hardware or software solutions to improve robot functionality by using external feedback and control.

Encouraged criteria:

- The solution should work consistently during most matches.
- The team should discuss reliability, either by demonstrated effectiveness or by identifying how the solution could be improved.
- The team should show use of the engineering process for sensors, hardware, and/or algorithms, including lessons learned.

Critical wording for this season: the portfolio must summarize software, sensors, and mechanical control, but should not include copies of code. Older seasons used a separate Control Award submission form; the current 2025-2026 manual language focuses on the portfolio. Do not assume a separate two-page form is required unless the local event specifically asks for one.

## What Strong Control Award Writing Should Do

A strong Control Award section should answer these questions quickly:

- What control problems did the robot have to solve?
- What sensors or feedback signals are used?
- What algorithm or control strategy transforms feedback into action?
- What mechanical system is being controlled?
- How does this improve match performance?
- What reliability protections are included?
- What did the team learn or improve after testing?

Good structure for each feature:

1. Challenge
2. Feedback source
3. Control method
4. Robot action
5. Match value
6. Reliability / lesson learned

Good evidence formats:

- Compact table of control systems.
- One system architecture diagram.
- One pathing or state-machine diagram.
- One tuning/result chart if measurements exist.
- Short before/after note, for example "unthrottled telemetry slowed the loop; competition telemetry now publishes at 5 Hz."

Avoid:

- Large code blocks.
- Explaining Java syntax.
- Listing every class.
- Claiming reliability without evidence or an improvement plan.
- Treating Control as autonomous-only. The manual allows TELEOP control solutions too.

## PortfolioLab Takeaways

FTCPortfolioLab describes itself as an educational platform for creating clear, structured, competition-ready FTC engineering portfolios. Its portfolio page provides rated benchmark portfolios and community PDF submissions with filters by season, competition level, and award, including Control.

Useful way to use it:

- Use it as an example library for layout, density, and judge-facing language.
- Filter by Control Award to compare how teams describe software and sensors.
- Treat it as inspiration, not official rules. The official source remains the current FIRST Competition Manual.

## Portfolio vs Engineering Notebook

Portfolio:

- Short, selective, judged-award-facing.
- Built around current criteria.
- Must fit inside 15 content pages.
- Should be readable without opening links or reading code.
- Should tell judges what matters and why it improves the robot.

Notebook:

- Longer, internal and technical.
- Can contain design history, implementation details, tuning notes, diagrams, experiments, failures, and links to source files.
- Can preserve knowledge for future programmers.
- Can support pit interviews when judges ask for more detail.

For this team, the portfolio should summarize the best control systems. The notebook should explain the full programming department: architecture, autonomous, TeleOp, subsystems, sensors, motor control, telemetry, testing, and next steps.

## Codebase Evidence Collected

Important files inspected:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/auto/NewAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/PPConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Transfer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/ColorSensors.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/HardwareManager.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryHub.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/telemetry/TelemetryCollector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/facade/VelocityControlledMotor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/facade/ProfiledPositionMotor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/PIDFFVelocityController.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/TrapezoidalMotionProfileController.java`
- Existing docs: `doc/FTC_Telemetry_Management_Plan.md`, `doc/FTC_Loop_Optimization_Report.md`, `doc/TurretController.md`, `doc/POSITION_LUT_AIMING.md`

## Control Systems Found in the Code

### 1. Pedro Pathing Autonomous

- Uses Pedro Pathing `Follower`, Bezier lines/curves, path chains, and goBILDA Pinpoint localization.
- `NewAuto` builds paths for preload scoring, pickups, gate interaction, repeated scoring, and parking.
- The autonomous routine is state-machine based. State transitions use follower busy status, timers, shooter readiness, and transfer/flicker timing.
- Alliance mirroring converts the same autonomous plan between red and blue.

Control Award angle: this is a feedback-driven autonomous system. Localization feedback lets the robot follow planned paths, while the state machine synchronizes movement with shooter and transfer mechanisms.

### 2. Field-Relative TeleOp and Localization

- `PPConstants` builds a Pedro Pathing follower with mecanum drivetrain constants and a Pinpoint localizer.
- TeleOp starts from an autonomous-stored pose if available.
- The IMU heading is read in TeleOp, and the follower updates robot pose and velocity.
- Manual bulk caching is enabled through `HardwareManager`, which clears hub caches once per loop.

Control Award angle: the robot maintains a field pose estimate that supports path following, aiming, and driver control.

### 3. Turret Aiming

- The turret can aim from field pose using `atan2` to calculate the angle from robot to goal, subtracting robot heading.
- The turret has angle limits from -100 deg to 120 deg.
- It uses an external gear ratio of 2.8 and converts mechanism angle into motor-side angle.
- Normal aiming uses a profiled position motor backed by a trapezoidal motion-profile controller and PIDF.
- The controller uses position error, velocity error, static feedforward, velocity feedforward, acceleration feedforward, and battery-voltage scaling.
- There is also a Limelight aiming mode that uses target horizontal offset (`tx`) and a manual PID loop.
- There is an experimental position-based aim LUT that can select virtual aim points based on robot position.
- Moving-shot compensation currently offsets the robot pose using velocity and distance; the existing turret review notes this is a useful attempt but could be improved by using tangential velocity and projectile flight time.

Control Award angle: this is one of the strongest control stories. It connects localization, vision, motor encoders, feedforward, motion profiling, and mechanical angle limits to make scoring more accurate.

### 4. Shooter Velocity Control

- The shooter uses a custom velocity-controlled motor facade.
- Target velocity changes between close and far presets in TeleOp and autonomous.
- PIDF velocity control uses encoder feedback and battery-voltage compensation.
- A follower shooter motor mirrors the main shooter power.
- The shooter measures filtered velocity with an exponential filter.
- Impact detection watches for a rapid velocity drop, which can indicate an artifact shot.
- The hood servo adjusts shot angle.
- Shooter readiness is based on measured velocity being close to target.

Control Award angle: flywheel control is external-feedback control. The code uses encoder velocity to maintain shot consistency and avoid shooting before the mechanism is ready.

### 5. Transfer and Intake Safeguards

- Transfer has explicit `STOP`, `SHOOT`, and `COLLECT` states.
- Three distance/color sensors detect whether artifacts are present.
- The color sensors update at 5 Hz rather than every loop.
- If the transfer is collecting and the sensors indicate full, or the intake overcurrent alert trips, collection stops.

Control Award angle: this is intelligent mechanism protection. Sensor feedback prevents overfilling and overcurrent conditions, improving reliability and driver workload.

### 6. Telemetry Architecture

- Telemetry is organized around `TelemetryProvider`, `TelemetryCollector`, and `TelemetryHub`.
- Modes are `OFF`, `COMPETITION`, `DEBUG`, and `TRACE`.
- Data fields are tagged with cost classes: `STATIC`, `CHEAP`, `BULK_CACHED`, `NON_BULK`, and `FORMATTED`.
- Competition telemetry publishes more slowly at 0.2 s intervals, while debug and trace can publish every loop.
- Expensive values, such as current draw, are throttled with `ThrottledValue`.
- Shooter and turret expose telemetry snapshots instead of directly formatting everything inside the control loop.

Control Award angle: this is a reliability and debugging system. It keeps the match loop focused on robot control while preserving visibility during testing.

### 7. Motor-Control Architecture

- `VelocityControlledMotor` packages encoder conversion, measured velocity, acceleration estimate, target velocity, and PIDFF output.
- `ProfiledPositionMotor` packages encoder conversion, target angle, limits, tolerance, and a trapezoidal motion profile.
- `PIDFFVelocityController` combines feedback and feedforward terms.
- `TrapezoidalMotionProfileController` creates a reference position, velocity, and acceleration so mechanisms move smoothly instead of simply slamming toward a setpoint.
- `LoopState` passes loop `dt` and battery-voltage factor into controllers.

Control Award angle: this is reusable mechanism control. The team created a control layer that can be reused across mechanisms instead of writing one-off motor code everywhere.

## Best Control Award Features to Highlight

Highest value for the portfolio:

1. Pose-based turret aiming with profiled PIDF and feedforward.
2. Shooter velocity control with readiness and filtered impact detection.
3. Pedro Pathing autonomous state machine with Pinpoint localization.
4. Sensor-protected transfer/intake using distance sensors and current alerts.
5. Telemetry modes and cost-based throttling for reliable debugging without slowing matches.

Possible one-page portfolio layout:

- Top half: system diagram or table of control components.
- Middle: three deep-dive cards: autonomous pathing, turret/shooter, intake/telemetry reliability.
- Bottom: testing and lessons learned.

## Best Programming Notebook Topics

Programming notebook sections should be more complete than the portfolio:

- Programming goals and match strategy.
- Software architecture overview.
- Autonomous architecture.
- TeleOp architecture.
- Localization and path following.
- Turret aiming and motion profiling.
- Shooter velocity control and shot detection.
- Transfer/intake sensor logic.
- Motor package abstraction.
- Telemetry architecture.
- Performance and loop optimization.
- Testing/tuning procedure.
- Lessons learned and planned improvements.

## Specific Lessons Learned to Mention

- Telemetry can reduce loop speed if everything is published every cycle, so the code now uses modes, publish intervals, and throttled expensive reads.
- Non-bulk current readings are treated differently from cheap/bulk-cached alerts.
- Manual bulk caching helps control loop speed when the cache is cleared once per loop.
- Distance/color sensors do not need to update every loop; the transfer sensors are throttled to 5 Hz.
- Turret aiming benefits from motion profiling because a smooth reference is easier to control than raw position error.
- Moving-shot turret compensation exists but should become more physics-based by separating tangential and radial velocity.
- Battery-voltage compensation helps shooter and turret feedforward stay more consistent as the battery drops.
- Autonomous reliability comes from state transitions that wait for robot path completion, shooter readiness, and timed transfer actions rather than assuming fixed timing for everything.

## Recommended Evidence to Add Later

If the team has time, collect these measurements:

- Shooter spin-up time for close and far presets.
- Shooter velocity error before and after PIDF tuning.
- Turret target error in degrees from several field positions.
- Autonomous completion rate over repeated practice runs.
- Loop frequency in `COMPETITION` vs `DEBUG` telemetry mode.
- Intake full-detection success rate and false-stop rate.

These numbers would make the Control Award section much stronger because they turn software claims into evidence.

