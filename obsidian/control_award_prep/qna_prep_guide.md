# FTC Control Award Q&A Preparation Guide

This guide compiles standard FTC programming questions, specific questions judges will ask based on your codebase, and advanced technical answers that reference the content on **pages 10, 11, and 12** of your Engineering Portfolio.

---

## 1. General FTC Judging Questions

### Q1: "What external libraries or pre-programmed code did your team use?"
* **How to answer:** Highlight where you built on top of libraries versus where you wrote custom implementations, referencing your portfolio diagrams.
* **Your Answer:** 
  > "As illustrated in our Software Architecture block diagram on **page 10** of our portfolio, we use **Pedro Pathing** for path generation (using Bezier curves) and localizer communication, and the **GoBilda Pinpoint Driver** for high-frequency hardware odometry. On top of that, we wrote our own custom subsystem layer including a non-blocking LED state machine driver, a custom profiled motor wrapper (`MetaMotor`), and our own multi-dimensional interpolation engines for turret and hood control."

### Q2: "How did you tune your PID and Feedforward coefficients?"
* **How to answer:** Explain the *methodology* (don't just say "trial and error"). Mention mathematical characterization and point to the data page.
* **Your Answer:**
  > "We characterized our turret and shooter motors using linear regression. As documented in our testing section on **page 12**, for the turret feedforward, we collected steady-state velocity data at various voltage levels. The slope of the resulting linear regression gave us our velocity feedforward constant ($k_v$), and the intercept gave us our static friction threshold ($k_s$). Once feedforward handled 90% of the control effort, we tuned our PID controller using Ziegler-Nichols method to correct transient disturbances."

---

## 2. Codebase-Specific Questions (The "Baits")

### Q3: "Why does the turret aim at a 'virtual target' instead of the real goal?"
* **Your Answer:**
  > "Due to physical asymmetries in our robot's shooter mounting, gyro misalignment, and gyroscopic precession when shooting on the move, a direct geometric line-of-sight vector to the goal does not guarantee a hit from all field coordinates. 
  > 
  > To solve this, we mapped the field and created a position-to-virtual-target lookup table. The table maps the robot's coordinates $(X,Y)$ to a compensated target coordinate. If the ball tends to drift left when shooting from the right side, the virtual target automatically shifts right to compensate. Our turret aims at this interpolated virtual target, ensuring shots enter the center of the net from anywhere on the field. You can see the 2D aim target map on **page 11** of our portfolio."

### Q4: "How does your 2D Hood LUT work, and why do you normalize the inputs?"
* **Your Answer:**
  > "Our hood angle depends on two inputs: distance from the goal (inches) and current flywheel velocity (ticks/second). We use Inverse Distance Weighting (IDW) interpolation among the 4 nearest calibration points. 
  > 
  > However, distance values range from 30 to 140, whereas velocity values range from 1000 to 1500. If we computed standard Euclidean distance, a change of 10 ticks/second (negligible) would dominate a change of 10 inches (massive). To prevent this unit scale bias, our code dynamically normalizes both inputs by dividing each delta by the total span of that variable in our sample database:
  > $$\text{distance}_{\text{norm}} = \frac{\Delta x}{x_{\text{max}} - x_{\text{min}}}$$
  > This allows the interpolation engine to weigh distance and velocity changes equally. The mathematical breakdown of this normalization is illustrated on **page 11**."

### Q5: "How does your shooter detect impacts/shots without a physical sensor?"
* **Your Answer:**
  > "To avoid adding mechanical complexity or potential sensor failures in the shooter barrel, we use **software-based impact detection**. 
  > 
  > In `Shooter.java`, we pass the raw flywheel velocity through an exponential low-pass filter to smooth out sensor noise: 
  > $$v_{\text{filtered}} = \alpha \cdot v_{\text{measured}} + (1 - \alpha) \cdot v_{\text{filtered\_prev}}$$
  > We then calculate the derivative (deceleration) of this filtered velocity. When a ring enters the flywheel, it draws energy, causing a sharp deceleration spike. If this deceleration rate exceeds our threshold, we trigger a shot-detected event, which immediately signals the driver via the LEDs and advances our autonomous state machine. You can see the telemetry plot of this deceleration drop on **page 12**."

### Q6: "How does your turret home itself without limit switches?"
* **Your Answer:**
  > "We use **stall-detection homing**. During initialization, the turret motor runs backward at a low, safe power. The code continuously checks if the motor current exceeds our overcurrent threshold (`turretHardware.isOverCurrent()`). Once the turret hits the physical hard stop, the motor stalls, causing a current spike. The code instantly cuts power, stops the motor, resets the encoder ticks to zero, and transitions to run-to-position mode. This gives us sub-degree homing accuracy with zero sensor overhead."

### Q7: "How do you handle battery voltage drop during a match?"
* **Your Answer:**
  > "Motors spin slower as the battery drains, which would ruin our shooter consistency and turret motion profile timings. We sample the hardware voltage sensor every 100ms. In both the profiled turret controller and the shooter feedforward loop, we scale our output power by multiplying the calculated target voltage by a compensation factor of $\frac{12.0}{V_{\text{measured}}}$. This ensures the motors receive the exact same effective voltage whether the battery is at 14V or 11V."

---

## 3. Explaining the "Red Alliance Bug" (Showcasing Engineering Integrity)

Judges love teams that talk about their testing process, bugs they encountered, and how they fixed them. If they ask about your testing process or a challenge you overcame, use this story:

* **The Challenge:**
  > "During our pre-scrimmage software audit, we reviewed our shooter distance-to-goal calculations. We discovered a critical coordinate system bug in our `ShooterHoodLuts` class: the distance calculation was hardcoded to use the Blue goal pose and ignored the alliance color parameter. On the Red alliance, this caused our robot to calculate distance to the opposite side of the field, leading to severe shooting errors."
* **The Resolution:**
  > "We updated the class to dynamically mirror the goal pose based on the alliance color, ensuring our lookup tables receive the correct field-centric distance regardless of which alliance color is selected on the driver station. This taught us the importance of writing rigorous unit tests for coordinate transformations."
