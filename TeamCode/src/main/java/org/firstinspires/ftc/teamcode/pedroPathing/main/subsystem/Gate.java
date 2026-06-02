package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    public Gate() {}
    private static final double SERVO_WRITE_EPSILON = 1e-4;
    private final double GATE_ACTIVATION_POS = 1.0;
    private final double GATE_REST_POS = .5;
    Servo gate;
    private double lastGatePosition = Double.NaN;
    public void init(HardwareMap hwmap) {
        gate = hwmap.get(Servo.class, "rightFlicker");
        gate.setDirection(Servo.Direction.FORWARD);
        applyGatePosition(GATE_REST_POS);
    }
    public void activate() { applyGatePosition(GATE_ACTIVATION_POS); }
    public void deactivate() { applyGatePosition(GATE_REST_POS); }
    public boolean isActivated() { return lastGatePosition == GATE_ACTIVATION_POS; }
    public void changeState() {
        if (isActivated()) {
            deactivate();
        }
        else {
            activate();
        }
    }
    private void applyGatePosition(double position) {
        if (Double.isNaN(lastGatePosition)
                || Math.abs(position - lastGatePosition) >= SERVO_WRITE_EPSILON) {
            gate.setPosition(position);
            lastGatePosition = position;
        }
    }
}
