# Control Award Portfolio Draft

This draft is written as judge-facing portfolio language. It avoids code copies and focuses on challenge, feedback, control method, and match impact.

## Control Award: Software and Sensor Control

Our robot uses feedback control to make scoring more repeatable in both autonomous and TeleOp. The main control systems are autonomous path following, pose-based turret aiming, shooter velocity regulation, sensor-protected transfer, and match/debug telemetry.

| Control component | Challenge solved | Feedback used | How it works | Match impact |
| --- | --- | --- | --- | --- |
| Pedro Pathing autonomous | Move through the field and score without driver input | goBILDA Pinpoint odometry, follower pose, timers, subsystem states | Autonomous is a state machine. Each state follows a Bezier path or path chain, then waits for conditions such as follower completion, shooter readiness, or transfer timing before moving on. | Makes autonomous scoring more consistent than fixed-time driving and lets movement coordinate with mechanisms. |
| Pose-based turret aiming | Aim at the goal while the robot is in different field positions | Robot pose, robot heading, turret encoder, optional Limelight `tx` | The robot calculates the angle from its field pose to the goal, converts that into a turret angle, clips it inside mechanical limits, and uses a profiled PIDF controller to move smoothly to the target. | Reduces driver aiming load and improves shot alignment across the field. |
| Shooter velocity control | Keep artifact shots consistent as battery and load change | Shooter encoder velocity and battery voltage | A PIDF velocity controller compares target velocity to measured velocity. Feedforward is scaled using battery voltage, and the robot waits until the shooter is near target before treating it as ready. | Improves shot consistency and prevents firing before the flywheel reaches speed. |
| Transfer/intake protection | Avoid overfilling and reduce jams/current issues | Three distance/color sensors and intake overcurrent alert | The transfer runs as `COLLECT`, `SHOOT`, or `STOP`. During collection, sensor fullness or overcurrent automatically stops intake. Sensor reads are throttled to avoid wasting loop time. | Protects the mechanism and makes collection less dependent on driver reaction time. |
| Telemetry modes | Debug control systems without slowing match code | Structured telemetry snapshots, cost classes, throttled current reads | Telemetry is separated into `COMPETITION`, `DEBUG`, and `TRACE` modes. Expensive values such as current draw are throttled, while competition mode shows only essential driver and fault information. | Keeps the control loop focused during matches while still giving programmers detailed data during testing. |

## Main Control System: Turret and Shooter

The strongest control system on our robot is the scoring stack: the drivetrain estimates field position, the turret aims from that pose, and the shooter regulates flywheel speed before firing.

The turret uses field geometry instead of only driver alignment. From the robot pose, the software computes the direction to the goal with trigonometry, subtracts robot heading, and sends the result to the turret controller. The turret angle is limited to the safe mechanical range of -100 degrees to 120 degrees. Because the turret has an external gear ratio, the software converts turret angle into motor angle before control.

Instead of commanding raw motor power from position error, the turret follows a trapezoidal motion profile. The controller generates a reference position, velocity, and acceleration, then uses PIDF control to follow that reference. The feedforward terms help overcome static friction and support planned velocity and acceleration. This makes aiming smoother and more repeatable, especially when the target angle changes quickly.

The shooter uses encoder velocity feedback. The driver can choose close or far shot presets, and the software changes flywheel velocity and hood angle together. The robot detects when the shooter is still busy by comparing measured velocity to target velocity. A filtered velocity signal is also used to detect sudden velocity drops after a shot, giving the team a way to understand shot timing and flywheel recovery.

## Autonomous Control

Our autonomous routine is built as a sequence of states rather than one long time-based script. The robot follows Pedro Pathing paths made from Bezier lines and curves. After each path or mechanism action, the next state begins only after useful feedback conditions are met, such as:

- the path follower is no longer busy,
- the shooter is no longer busy,
- a transfer/flicker timer has completed,
- the robot has finished a collection or scoring action.

This makes the routine more reliable because the robot does not assume every movement and mechanism takes the same time on every run. The same path plan can also be mirrored for the opposite alliance, reducing duplicate code and making tuning easier.

## Reliability and Lessons Learned

A major lesson this season was that control quality depends on loop consistency, not only on the algorithm. We enabled manual bulk caching for the hubs, throttled slow sensor reads, and separated competition telemetry from debug telemetry. This reduced unnecessary work in the hot loop while keeping the data needed for tuning.

We also learned that some feedback values are cheap and useful every loop, while others should be sampled more carefully. For example, overcurrent alerts are useful as competition-safe fault signals, but continuous current readings are slower and are kept in debug telemetry with throttling.

Our next improvement is to make moving-shot turret compensation more physics-based. The current version uses robot velocity and distance to lead the aim point. The next version should separate tangential and radial velocity and use estimated projectile flight time, which would make aiming while moving more accurate across different field positions.

## Suggested Visuals for the Portfolio Page

- One block diagram: Pinpoint/IMU/Limelight/sensors -> control algorithms -> drivetrain/turret/shooter/transfer.
- One small autonomous state-machine diagram.
- One shooter or turret tuning graph if available.
- One table of "control feature, feedback, reliability protection."

