package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients;

public class MotionProfilingCoefficients {
    PIDFFCoefficients pidCoef; double maxVelocity, maxAcceleration;
    public MotionProfilingCoefficients(double kp, double ki, double kd, double kf, double kv, double ka, double maxVelocity, double maxAcceleration) {
        this.pidCoef = new PIDFFCoefficients(kp, ki, kd, kf, kv, ka);
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
