# Turret Controller Review

The turret uses a **motion-profiled PIDF position controller** to choose motor power during normal aiming.

Relevant code:
[Turret.java](/C:/Users/Lefteris%20Dragasakis/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java)
[TrapezoidalMotionProfileController.java](/C:/Users/Lefteris%20Dragasakis/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/TrapezoidalMotionProfileController.java)
[PIDFFPositionController.java](/C:/Users/Lefteris%20Dragasakis/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/motor/math/controllers/PIDFFPositionController.java)
[RobotConstants.java](/C:/Users/Lefteris%20Dragasakis/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/constants/RobotConstants.java)

## 1. How power is chosen

When you call `setAngleRadians()`, `lookToGoal()`, or `lookToGoalWhileMoving()`, the turret does **not** jump straight to a raw PID correction.  
Instead, it first creates a **moving reference** that obeys speed and acceleration limits, and then drives the motor toward that reference.

### Step 1: pick a target angle

The requested mechanism angle is clipped to:

- minimum: `-100 deg`
- maximum: `120 deg`

So the controller never tries to command beyond those angular limits.

### Step 2: convert mechanism angle into motor-space angle

Because the turret has an external gear ratio of `2.8`, the motor must rotate more than the turret itself.  
So the controller tracks the motor-side angle, not just the turret-side angle.

That means:

- 1 turret radian becomes `2.8` motor radians
- the controller is effectively working on the geared motor side

### Step 3: generate a trapezoidal-style motion reference

The profiled controller keeps an internal reference state:

- reference position `xRef`
- reference velocity `vRef`
- reference acceleration `aRef`

Instead of asking “how far am I from target?” and instantly converting that into power, it asks:

- how much distance is left?
- am I far enough away to keep accelerating?
- or am I close enough that I must decelerate now to stop cleanly?

It computes a **stopping distance**:

\[
stoppingDistance = \frac{vRef^2}{2 \cdot maxAcceleration}
\]

Then it decides acceleration:

- if it is already moving toward the goal and the remaining distance is smaller than the stopping distance, it **decelerates**
- otherwise it **accelerates toward the target**

Your profile limits come from `RobotConstants.TURRET_PROFILE_COEFFICIENTS`:

- max velocity: `1800 ticks/s`
- max acceleration: `4500 ticks/s^2`

After conversion to motor-angle units, that is about:

- `21.03 rad/s` max reference velocity
- `52.58 rad/s^2` max reference acceleration

So the controller is trying to move fast, but only within those motion limits.

### Step 4: turn that reference into motor power with PIDF

Once the moving reference is chosen, power is computed as:

\[
power = PID + Feedforward
\]

with:

\[
PID = k_p \cdot error + k_i \cdot integral + k_d \cdot velocityError
\]

and

\[
FF = k_s \cdot sign + k_v \cdot vRef + k_a \cdot aRef
\]

Where:

- `error = referencePosition - measuredPosition`
- `velocityError = referenceVelocity - measuredVelocity`

This is important: your `D` term is not “difference in position error over time”.  
It is effectively a **velocity error term**. That makes it act more like damping against mismatch between desired speed and actual speed.

### The gains used in normal profiled mode

From `RobotConstants` the legacy turret PIDF values are:

- `kp = 0.068`
- `ki = 0`
- `kd = 0.002`
- `ks = 1.2`
- `kv = 0.005687094208999908`
- `ka = 0.0004`

These are scaled from tick-space into angular units before use. After conversion, the effective controller values are about:

- `kp = 5.82`
- `ki = 0`
- `kd = 0.171`
- `ks = 1.2`
- `kv = 0.487`
- `ka = 0.0342`

So in normal mode, power comes from four ideas at once:

- position error pushes you toward target
- velocity mismatch damps or boosts motion
- static feedforward helps break friction in the needed direction
- velocity and acceleration feedforward “pre-pay” the expected effort for the planned motion

### Step 5: battery compensation

The feedforward part is multiplied by:

\[
1 / batteryVoltage
\]

So if battery voltage drops, the feedforward contribution becomes larger.  
That helps keep the same physical response even when the robot battery sags.

### Step 6: clip final power

After all of that, the final output is clipped to the turret’s max power:

- `[-1.0, 1.0]`

So even if the controller asks for more, the motor never receives more than full allowed power.

## 2. How `lookToGoal()` affects the controller

`lookToGoal()` computes the angle from robot pose to the scoring goal using `atan2`, subtracts robot heading, and sends that angle into the profiled controller.

So the power selection chain becomes:

1. compute desired turret angle to the goal
2. clip it into allowed angular range
3. convert to motor-space angle
4. generate a motion-limited reference
5. compute PIDF power to follow that reference

That means the turret does **not** directly power itself based on goal angle error alone.  
It powers itself based on how far the **current motor state** is from the **reference trajectory**.

## 3. The Main Weakness Is Lead Compensation, Not the Controller

The turret controller itself is structurally sound.

- The profiled controller already uses a proper position-plus-velocity-error control law.
- The `kd` contribution is effectively a **velocity error term**, which is the right kind of damping for this setup.
- The feedforward is also structured correctly: static, velocity, and acceleration terms are all present.

So the main issue is **not** the power-selection logic inside the controller.  
The weak point is the way moving-shot compensation currently changes the target angle.

### What the current moving-shot compensation does

`lookToGoalWhileMoving()` shifts the robot pose before aiming:

\[
compensatedPose = pose + leadFactor \cdot velocity \cdot distance
\]

with default:

- `movingShotLeadFactor = 0.01`

This means the aim correction is created by translating the robot pose in the same direction as its full velocity vector, scaled by distance.

### Problem 1: it does not separate radial and tangential motion

The full robot velocity is not equally important for aiming.

- **Tangential velocity** relative to the goal changes the required aim angle.
- **Radial velocity** toward or away from the goal mainly changes distance, not the direction you should point the turret.

So when the code multiplies the entire velocity vector by distance, it applies lead in directions that should not contribute directly to angular correction.

Conceptually:

```text
       GOAL
        |
        |  <- radial motion changes range more than aim angle
        |
       [R] -> tangential motion is what really shifts aim angle
```

Because of that, the current correction can:

- under-correct when moving sideways
- over-correct when moving toward the goal
- over-correct in the opposite sense when moving away
- mix both errors during diagonal motion

### Problem 2: the lead factor is a tuning number, not a physics-based time model

The current formula depends on:

- `leadFactor`
- robot velocity
- distance

but it does **not** explicitly model projectile flight time.

The physically meaningful quantity is the angle required for the turret to account for how far the robot will move **while the projectile is in the air**.

That lead angle is more naturally described as:

\[
leadAngle = \operatorname{atan2}(v_{tangential} \cdot flightTime,\ distance)
\]

where:

\[
flightTime \approx \frac{distance}{projectileSpeed}
\]

or even better, whatever flight-time estimate already comes from your shooter lookup/simulation data.

That matters because the current `0.01` factor is a single tuned constant, while the real required lead should change with:

- distance
- projectile speed
- tangential robot speed

So even if `0.01` feels correct at one distance, it is unlikely to stay correct across the full field.

### Why this matters even if the chassis is not rotating

Even with zero chassis rotation, the turret can still miss because the moving-shot correction is not physically grounded.

Examples:

- Moving sideways fast: the current method can apply too little angular correction because part of the correction is spent in the wrong direction.
- Moving toward the goal: the current method can create excessive correction because radial motion is being folded into the lead translation.
- Moving away from the goal: similar error, but in the opposite sense.
- Moving diagonally: both effects combine.

### What the better correction should look like

The cleaner approach is:

1. Compute the vector from robot to goal.
2. Normalize it to get the goal direction.
3. Project robot velocity into:
   - tangential component
   - radial component
4. Use a flight-time estimate from your existing shooter data.
5. Convert the tangential motion during flight into an angular offset.
6. Add that angular offset directly to the target angle.

That means the lead should ideally be applied as an **angular offset**, not as a pose translation.

In practical terms, the target would be built from:

- base angle to goal
- minus robot heading
- plus a physics-based lead angle

### What this means for the controller

The important distinction is:

- the **controller** is choosing motor power in a sensible way
- the **target angle** being fed into that controller is where the main aiming error can be introduced

So if shots miss while moving, the first thing to question is not the PIDF architecture.  
It is the moving-shot lead model.

## 4. What “the right amount of power” means in your code

Your code decides the “right” power as the amount that best follows a planned motion.

It is the sum of:

- enough power to reduce position error
- enough power to match desired velocity
- enough power to overcome static friction
- enough power to support the planned velocity and acceleration
- all while respecting max velocity, max acceleration, angle limits, and power clipping

So here, “right power” means:

- not just “go toward target”
- but “follow a smooth physically limited path to target”

## 5. Controller-specific details worth noticing

- `ki = 0`, so there is **no integral action** in the profiled controller.
- The static feedforward sign comes from reference velocity, and if that is zero, it falls back to the sign of remaining position error. That helps it still choose a direction to break static friction near rest.
- The controller resets when the turret re-enters profiled mode, so it does not carry old internal state into a fresh aiming command.
