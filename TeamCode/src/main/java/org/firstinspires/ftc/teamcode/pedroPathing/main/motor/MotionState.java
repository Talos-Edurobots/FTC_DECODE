package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public final class MotionState {
    public double position;
    public double velocity;
    public double acceleration;

    public void set(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}

