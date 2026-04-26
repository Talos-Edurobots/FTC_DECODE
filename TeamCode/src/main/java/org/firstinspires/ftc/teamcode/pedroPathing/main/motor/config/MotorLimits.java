package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config;

public final class MotorLimits {
    private final double maxPower;
    private final double currentAlertAmps;

    public MotorLimits(double maxPower, double currentAlertAmps) {
        this.maxPower = maxPower;
        this.currentAlertAmps = currentAlertAmps;
    }

    public static MotorLimits defaults() {
        return new MotorLimits(1.0, Double.POSITIVE_INFINITY);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public double getCurrentAlertAmps() {
        return currentAlertAmps;
    }
}
