package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public final class MotionState {
    private double position;
    private double velocity;
    private double acceleration;


    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void set(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}

