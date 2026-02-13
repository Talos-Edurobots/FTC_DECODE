package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;

public class MotionProfilingController {
    MotionProfilingCoefficients coef;
    public MotionProfilingController(MotionProfilingCoefficients coefficients, MotorUse use) {;
        this.coef = coefficients;
    }
//    public double update(double target, MotionState motionState, LoopState loopState) {
//        double position = motionState.position;
//        double velocity = motionState.velocity;
//
//        double remaining = target - xRef;
//
//        double stoppingDistance =
//                (vRef * vRef) / (2.0 * maxAcceleration);
//
//        if (Math.abs(remaining) <= stoppingDistance) {
//            aRef = -Math.signum(velocity) * maxAcceleration;
//        } else {
//            aRef = Math.signum(remaining) * maxAcceleration;
//        }
//
//        vRef += aRef * dt;
//        vRef = Range.clip(vRef, -maxVelocity, maxVelocity);
//
//        xRef += vRef * dt;
//
//        if (Math.signum(targetPositionTicks - xRef)
//                != Math.signum(remaining)) {
//            xRef = targetPositionTicks;
//            vRef = 0.0;
//            aRef = 0.0;
//        }
//
//        double positionError = xRef - position;
//        double velocityError = vRef - velocity;
//
//        double pidVolts =
//                kP * positionError +
//                        kD * velocityError;
//
//        double ffVolts =
//                kS * Math.signum(vRef) +
//                        kV * vRef +
//                        kA * aRef;
//
//        double power =
//                (pidVolts + ffVolts) / batteryVoltage;
//    }
}
