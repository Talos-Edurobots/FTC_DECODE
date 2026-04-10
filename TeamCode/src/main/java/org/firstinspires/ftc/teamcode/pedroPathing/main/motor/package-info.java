/**
 * Motor-control abstractions for FTC mechanisms.
 *
 * <p>This package is moving away from a single universal motor object and toward
 * explicit composition:
 *
 * <p>subsystem -> facade -> controller -> MetaMotor
 *
 * <p>The package owns motor-shaft concepts such as encoder conversion, controller
 * math, loop state, and hardware access. Mechanism-specific conversion belongs in
 * the subsystem layer.
 */
package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;
