# Control Award Code & Portfolio Evaluation Report

This report evaluates the current judging notes and your engineering portfolio against the actual codebase of **FTC DECODE (Team #25070 - Talos EduRobots)**. It identifies discrepancies, highlights sophisticated software engineering features that are currently missing from your materials, and documents a critical alliance bug.

---

## 1. Executive Summary & Critical Findings

* **Current Notes Completeness Score: 4/10** (Due to major omissions in your portfolio).
* **Critical Finding: Blank Portfolio Pages 10, 11, and 12**
  Your uploaded engineering portfolio (`_Instabul Portfolio.pdf`) has **completely blank pages** under the headings **"Programming"** (page 10), **"Control Systems"** (page 11), and **"Data & Testing"** (page 12). 
  > [!IMPORTANT]
  > To even be considered for the Control Award, these pages **must** be filled. Judges rely heavily on the engineering portfolio to verify your code and control systems. We have written the exact content you should insert into these pages in the file `portfolio_content.md`.
* **Critical Finding: The Red Alliance Shooter Bug**
  In `ShooterHoodLuts.java`, the distance calculation is hardcoded to `BLUE_GOAL_POSE` and completely ignores the alliance color. This will cause major shooter errors on the Red Alliance.
* **Omitted Innovations:** Multiple advanced automations (sensorless shot detection, stall-based homing, active flywheel braking, LED feedback loops, and driver acceleration ramping) are completely missing from the notes and the portfolio.

---

## 2. Subsystem-by-Subsystem Technical Analysis

### A. Turret Control
* **In the Notes:** Mentions LUT tuning, motion profiling, and PID + Feedforward.
* **In the Code ([Turret.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java)):**
  * **Dynamic Aim Lead Compensation (Shooting on the Move):** The method `lookToGoalWhileMoving` uses the robot's current velocity vector and distance to the goal to project a forward-compensated target. This predicts and offsets the flight time of the shot while the robot is driving at full speed.
  * **Position-Based Virtual Targets (PositionAimLut):** Instead of aiming at the physical goal, the turret aims at a *virtual target* computed by a 2D position LUT using **Inverse Distance Weighting (IDW) interpolation** across the nearest $N$ (default 3) samples. This compensates for alignment drift, shooter spin axis offsets, or arena friction variations depending on where the robot is on the field.
  * **Stall-Detection Homing (Sensorless Calibration):** A homing routine where the turret runs backwards (`setPower(-0.5)`) and monitors motor current (`turretHardware.isOverCurrent()`). When it stalls against the hard stop, it resets the encoder. This eliminates the need for limit switches or magnetic homing sensors.
  * **Dual-Mode Auto-Switching:** The controller runs a trapezoidal profile using `ProfiledPositionMotor` and automatically switches to manual PID (`autoSwitchToPid`) once the error is within `pidSwitchThresholdDegrees` for a faster, tighter settle at the target.

### B. Shooter Control
* **In the Notes:** Mentions LUT tuning, PID/P + Feedforward, and battery voltage compensation.
* **In the Code ([Shooter.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java)):**
  * **Sensorless Shot/Impact Detection:** Uses digital signal processing (DSP) on the flywheel velocity. It runs an **exponential low-pass filter** (`filteredVelocity = alpha * measured + (1 - alpha) * filteredVelocity`) and tracks the velocity derivative (`delta`). A sudden deceleration spike (`-delta > dropThreshold`) triggers an impact event, signifying a shot was completed. This is used for driver LED feedback and state transitions.
  * **Active Brake-to-Float Speed Adjustment:** To slow down the flywheel quickly when targets change, the code changes zero-power behavior to `BRAKE` and sets power to `0`, then resets to `FLOAT` to prevent motor drag during shooting.
  * **Voltage-Compensated Feedforward:** Directly incorporates battery voltage scaling into the feedforward term to guarantee speed consistency.

### C. Hood Control
* **In the Notes:** Mentions a "two-dimensional lookup table based on both distance and flywheel velocity."
* **In the Code ([HoodAngleLut.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/HoodAngleLut.java)):**
  * **Euclidean Normalization for Multi-Unit Interpolation:** Because distance (inches) and velocity (ticks/second) have vastly different scales, calculating Euclidean distance directly would bias the nearest-neighbor search. The code resolves this by computing dynamic scale ranges (`distanceScale` and `velocityScale`) and normalizing the inputs before running **Inverse Distance Weighting (IDW)**. This is a very high-level math prep step.
  * **Nearest-Neighbor Interpolation:** Implements a custom 4-nearest-neighbor IDW algorithm to interpolate the 3D surface (distance, velocity $\rightarrow$ hood position).

### D. Transfer & Intake System
* **In the Notes:** Mentions 3 distance sensors, intake current monitoring, and automatic intake control.
* **In the Code ([Transfer.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Transfer.java) / [ColorSensors.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/ColorSensors.java)):**
  * **Debounced Sensor Fusion:** Uses 3 distance sensors (named `"color1"`, `"color2"`, `"color3"` in hardware). Rather than instantly reacting, it uses a temporal filter: `isFull()` requires all 3 sensors to detect a pixel for at least `0.4` seconds to filter out transient noises or passing objects.
  * **Intake Current Monitoring is Inactive:** The current-based jam detection (`intake.isOverCurrentForInterval(2)`) is currently **commented out** in `Transfer.java` (line 55). It is only used for telemetry readout.

---

## 3. Omitted Subsystems (Missing from Notes & Portfolio)
* **Driver Feedback System ([Leds.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Leds.java)):** A state-machine-controlled LED indicator system that uses PWM colors to give visual feedback to the driver when the shooter is spinned up, when transfer is full, and when climbing. This significantly reduces cycle times.
* **Drivetrain Slip Mitigation ([DriveTrain.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/DriveTrain.java)):** Implements an acceleration ramping function (`FieldCentricAccelerationDrive`) to smooth out manual inputs. This prevents wheel slippage, which is the primary cause of odometry drift in mecanum robots.
* **AprilTag Relocalization ([MainTeleOp.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java)):** Integrates Limelight 3A (MegaTag2 and MegaTag1) to dynamically correct the Pinpoint odometry pose estimator when the robot is nearly stationary.

---

## 4. Critical Code Bug / Vulnerability Alert

> [!WARNING]
> **The Red Alliance Shooter Calibration Bug**
> In `ShooterHoodLuts.java`, the method `distanceToGoal` is defined as:
> ```java
> public static double distanceToGoal(Pose robotPose, boolean isRed) {
>     return ShooterVelocityLut.distanceToGoal(robotPose, BLUE_GOAL_POSE);
> }
> ```
> Notice that `isRed` is passed as a parameter but **never used**. The distance is always calculated to the `BLUE_GOAL_POSE` (`new Pose(15.0, 128.0)`). 
> 
> * **Why this is a problem:** If the robot is on the Red alliance, its pose is mirrored to the red side of the field. The code will measure the distance from the red side all the way back to the blue goal on the opposite side of the field, resulting in a distance error of over 100 inches. This will cause the shooter lookup tables to output incorrect velocity and hood angles on Red.
> * **Suggested Fix:**
>   Modify the method to account for the alliance:
>   ```java
>   public static double distanceToGoal(Pose robotPose, boolean isRed) {
>       Pose goalPose = isRed ? new Pose(144.0 - BLUE_GOAL_POSE.getX(), BLUE_GOAL_POSE.getY()) : BLUE_GOAL_POSE;
>       return ShooterVelocityLut.distanceToGoal(robotPose, goalPose);
>   }
>   ```
