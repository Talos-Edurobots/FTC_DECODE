package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units;

public class Angle {
    private final double radians;

    public Angle(double radians) {
        this.radians = radians;
    }

    public static Angle fromRadians(double radians) {
        return new Angle(radians);
    }

    public static Angle fromDegrees(double degrees) {
        return new Angle(Math.toRadians(degrees));
    }

    public double toRadians() {
        return radians;
    }

    public double toDegrees() {
        return Math.toDegrees(radians);
    }

    public Angle add(Angle other) {
        return new Angle(this.radians + other.radians);
    }

    public Angle subtract(Angle other) {
        return new Angle(this.radians - other.radians);
    }
}
