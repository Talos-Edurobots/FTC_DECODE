# Scoring simulator

We built a Python simulator to calibrate and validate position-dependent aiming without repeatedly moving the physical robot.

The simulator:

- places the robot at field coordinates and heading
- visualizes the field, robot, goal, turret direction, and virtual aim point
- adjusts turret aim, flywheel velocity, and hood configuration
- stores calibrated robot-position/aim-point samples
- exports LUT samples for the Java match code

## Before/after experiment

The current simulator is interactive, so the next evidence procedure is to test and record the same pose grid twice:

| Configuration | Enabled behavior |
| --- | --- |
| Before | Fixed turret target and two fixed shooter/hood combinations |
| After | Interpolated turret aim point, linear distance-to-velocity LUT, and distance/velocity hood LUT |

Record each result, then generate field heatmaps and report the increase in scoreable area or scoreable cells. Use the same pose spacing, robot constraints, and success definition in both runs. Automating this grid sweep and heatmap export would make the procedure faster, but it is not implemented yet.

The simulator makes broad field testing fast and repeatable, but model assumptions can differ from the physical robot. Representative center, boundary, near, and far poses should therefore be spot-checked using the corrected Pinpoint/Limelight pose on the field. Relocalization accuracy itself is measured on the physical field at known poses.

See [[Turret LUT]] and [[TurretLUT_img.png]].
