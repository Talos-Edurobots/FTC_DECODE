Our main software goal was driver automation. The driver chooses when to collect and shoot. The robot handles aiming, shooter speed, hood angle, and transfer control.

For the turret, we tested multiple software versions. The final version uses the robot's position and an interpolated lookup table to calculate the turret angle from anywhere on the field. Motion profiling and PID plus feedforward make the movement smooth and accurate.

For the shooter, distance sets the flywheel velocity from a lookup table. Feedforward gives most of the needed power. Battery voltage compensation keeps it consistent, and proportional feedback corrects the remaining error.

The hood also uses a lookup table, based on distance and flywheel velocity. This keeps the shot trajectory more consistent.

Finally, the transfer uses three distance sensors and intake current monitoring. This helps automate intake control, reduce jams, and lower driver workload.
