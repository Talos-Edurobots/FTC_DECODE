# Writing

FTC Judges Presentation Notes (Software Section)

Current Technical Content

Turret Control

- Turret LUT tuning
- Motion profiling
- PID + Feedforward controller

Shooter Control

- Shooter LUT tuning
- PID/P + Feedforward controller

Hood Control

- Hood LUT

Transfer System

- 3 distance sensors
- Intake current monitoring
- Automatic intake control

---

Overall Evaluation

Technical Sophistication

9.5/10

The software demonstrates engineering practices that are significantly above average for FTC:

- Motion profiling
- Feedforward characterization
- Lookup tables
- Interpolation
- Battery voltage compensation
- Simulation-driven development
- Sensor-based automation

Communication Clarity

6/10

The content is technically strong but contains too many implementation details for a short judges presentation.

Presentation Suitability

7/10

The content would fit a 3–4 minute technical explanation, but the allotted time is only 1.5 minutes.

---

Language Fixes

Replace:

- "letted us visualize" → "allowed us to visualize"
- "postion" → "position"
- "calculated the slope a linear regression graph" → "calculated the slope of a linear regression"
- "making it the shots consistent" → "making shots consistent"

---

Recommended Focus

Judges care more about:

- Problems identified
- Engineering process
- Final solution
- Competitive benefit

Judges care less about:

- Exact controller equations
- Position/velocity/acceleration references
- Voltage-velocity pairs
- Detailed tuning procedures

---

Turret Section Recommendations

Current Issue

Too much time is spent describing historical implementations.

Current content:

«The 1st one used a limelight camera...»

«The 2nd one used the pinpoint...»

These details consume valuable presentation time.

Recommended Replacement

«We developed several turret iterations throughout the season, identified their limitations, and built a Python simulation that allowed us to create an interpolated LUT for accurate aiming from anywhere on the field.»

This is significantly shorter while preserving the engineering story.

LUT Benefit

Add:

«This allowed us to score accurately from a much larger area of the field.»

This communicates impact rather than implementation.

---

Motion Profiling Section

Current:

«We use a trapezoidal motion profile to generate position, velocity and acceleration references.»

Recommended:

«We use a trapezoidal motion profile and a PID plus feedforward controller for smooth and accurate turret movement.»

---

Feedforward Discussion

Main Presentation

Do not discuss regression details.

Recommended:

«We characterized the turret and obtained the velocity feedforward constant automatically.»

or

«We characterized the turret to obtain the velocity feedforward constant.»

Judge Questions

If a judge asks how feedforward was determined:

«We collected voltage-velocity data and used linear regression. The slope of the regression gives the velocity feedforward constant.»

This detail is better reserved for Q&A.

---

Shooter Section

Current approach is good.

Key points to mention:

- LUT converts distance to flywheel velocity.
- Values obtained experimentally.
- Interpolation generalizes the measurements.
- Feedforward provides most of the control effort.
- P term corrects errors.
- Battery voltage compensation improves consistency.

Suggested benefit statement:

«This gives us fast flywheel acceleration and reliable shooting consistency.»

---

Hood Section

Good concept.

Key point:

«The hood uses a two-dimensional lookup table based on both distance and flywheel velocity.»

Benefit:

«This keeps trajectories consistent even when flywheel speed changes during operation.»

---

Transfer Section

Strong section.

Mention:

- Three distance sensors track artifact positions.
- Intake current monitoring detects loading conditions.
- Automatic intake control reduces jams and driver workload.

---

Recommended Presentation Version

At the start of the season, we identified driver automation as a key requirement. As a result, the turret, shooter, hood, and transfer system operate fully autonomously.

For the turret, we developed multiple iterations throughout the season and identified limitations in our aiming accuracy. To solve this, we created a Python simulation that allowed us to visualize turret behavior and develop an interpolated lookup table. Using the robot's position, the LUT calculates the correct turret angle from anywhere on the field. The turret follows these targets using motion profiling and a PID plus feedforward controller, providing smooth and accurate movement.

For the shooter, we use another lookup table that converts distance to the required flywheel velocity. The values were obtained experimentally and generalized through interpolation. A feedforward controller with battery voltage compensation provides fast and consistent velocity control, while proportional feedback corrects remaining errors.

The hood uses a two-dimensional lookup table based on both distance and flywheel velocity. This allows the robot to maintain consistent trajectories even when flywheel speed changes during operation.

Finally, the transfer system uses three distance sensors and intake motor current monitoring to detect artifacts and automate intake control, reducing jams and driver workload.

---

Key Principle

For the presentation:

- Explain the problem.
- Explain the solution.
- Explain the benefit.

For judge questions:

- Explain the implementation details.
- Explain characterization methods.
- Explain regression, feedforward tuning, and control theory.