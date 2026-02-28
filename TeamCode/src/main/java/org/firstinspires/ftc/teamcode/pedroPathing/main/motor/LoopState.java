package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public final class LoopState {
    private double dt, batteryVoltageFactor;

    public double getDt() {
        return dt;
    }
    public double getBatteryVoltageFactor() {
        return batteryVoltageFactor;
    }

    public LoopState() {}
    public void set(double dt, double batteryVoltageFactor) {
        this.dt = dt;
        this.batteryVoltageFactor = batteryVoltageFactor;
    }
}
