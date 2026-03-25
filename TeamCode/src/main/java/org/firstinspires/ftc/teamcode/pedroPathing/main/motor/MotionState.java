package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;

public final class MotionState {
    private final Angle position;
    private final AngularVelocity velocity;
    private final double acceleration; // rad/s^2

    public MotionState(Angle position, AngularVelocity velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public Angle getPosition() {
        return position;
    }

    public AngularVelocity getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }
}
