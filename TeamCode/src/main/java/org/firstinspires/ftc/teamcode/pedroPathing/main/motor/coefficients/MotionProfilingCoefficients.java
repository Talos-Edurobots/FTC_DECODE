package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients;

public class MotionProfilingCoefficients {
    public PIDFFCoefficients getPidCoef() {
        return pidCoef;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    PIDFFCoefficients pidCoef; double maxVelocity, maxAcceleration, maxDeceleration;
    public MotionProfilingCoefficients(double kp, double ki, double kd, double kf, double kv, double ka, double maxVelocity, double maxAcceleration) {
        this(kp, ki, kd, kf, kv, ka, maxVelocity, maxAcceleration, maxAcceleration);
    }
    public MotionProfilingCoefficients(double kp, double ki, double kd, double kf, double kv, double ka, double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.pidCoef = new PIDFFCoefficients(kp, ki, kd, kf, kv, ka);
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }
    public MotionProfilingCoefficients(PIDFFCoefficients pidCoef, double maxVelocity, double maxAcceleration) {
        this(pidCoef, maxVelocity, maxAcceleration, maxAcceleration);
    }
    public MotionProfilingCoefficients(PIDFFCoefficients pidCoef, double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.pidCoef = pidCoef;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }

    public void setPidCoefficients(double kp, double ki, double kd, double ks, double kv, double ka) {
        pidCoef.set(kp, ki, kd, ks, kv, ka);
    }

    public void setMotionProfileLimits(double maxVelocity, double maxAcceleration) {
        setMotionProfileLimits(maxVelocity, maxAcceleration, maxAcceleration);
    }

    public void setMotionProfileLimits(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }
}
