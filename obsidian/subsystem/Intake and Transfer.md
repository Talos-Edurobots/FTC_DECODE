We use the intake motor, 3 color sensors and the gate on this subsystems. Its main functionality is to use state machine for different types of states the turret should be. There are 3 states:
### INTAKE state
- intake motor turns on
- gate activates
### SHOOT state
- intake motor turns on gate activates
### STOP state
- intake motor turns off
- gate activates

## Automations
when the robot has collected 3 artifacts, the intake motor turns off and allerts the driver using the LEDs. That makes the cycle times faster and the controls more automated. To make code flexible and telemetry easy, the motor is an object from the [[motor package]].