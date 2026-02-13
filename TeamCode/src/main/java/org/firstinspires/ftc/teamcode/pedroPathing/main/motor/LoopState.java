package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public final class LoopState {
    double dt, batteryVoltage;

    public double getDt() {
        return dt;
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public LoopState(double dt, double batteryVoltage) {
        this.dt = dt;
        this.batteryVoltage = batteryVoltage;
    }
}
