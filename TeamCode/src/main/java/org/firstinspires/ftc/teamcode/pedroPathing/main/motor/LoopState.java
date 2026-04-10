package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public final class LoopState {
    private static final double DEFAULT_BATTERY_VOLTAGE = 12.0;

    private double dt;
    private double batteryVoltageFactor = 1.0 / DEFAULT_BATTERY_VOLTAGE;

    public double getDt() {
        return dt;
    }

    public double getBatteryVoltageFactor() {
        return batteryVoltageFactor;
    }

    public double getBatteryVoltage() {
        if (batteryVoltageFactor <= 0.0) {
            return DEFAULT_BATTERY_VOLTAGE;
        }
        return 1.0 / batteryVoltageFactor;
    }

    public LoopState() {}

    public void set(double dt, double batteryVoltageFactor) {
        this.dt = dt;
        this.batteryVoltageFactor = batteryVoltageFactor;
    }

    public void setBatteryVoltage(double batteryVoltage) {
        if (batteryVoltage > 0.0) {
            this.batteryVoltageFactor = 1.0 / batteryVoltage;
        } else {
            this.batteryVoltageFactor = 1.0 / DEFAULT_BATTERY_VOLTAGE;
        }
    }
}
