package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units;

public class AngularVelocity {
    private final double radPerSec;

    public AngularVelocity(double radPerSec) {
        this.radPerSec = radPerSec;
    }

    public static AngularVelocity fromRadPerSec(double radPerSec) {
        return new AngularVelocity(radPerSec);
    }

    public static AngularVelocity fromRpm(double rpm) {
        return new AngularVelocity(rpm * 2 * Math.PI / 60.0);
    }

    public double toRadPerSec() {
        return radPerSec;
    }

    public double toRpm() {
        return radPerSec * 60.0 / (2 * Math.PI);
    }

    public AngularVelocity add(AngularVelocity other) {
        return new AngularVelocity(this.radPerSec + other.radPerSec);
    }

    public AngularVelocity subtract(AngularVelocity other) {
        return new AngularVelocity(this.radPerSec - other.radPerSec);
    }
}

