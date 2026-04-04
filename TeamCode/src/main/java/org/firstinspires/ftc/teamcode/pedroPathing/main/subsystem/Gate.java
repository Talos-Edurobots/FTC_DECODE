package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    public Gate() {}
    private final double GATE_ACTIVATION_POS = 1.0;
    private final double GATE_REST_POS = .5;
    Servo gate;
    public void init(HardwareMap hwmap) {
        gate = hwmap.servo.get("gate");
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(GATE_REST_POS);
    }
    public void activate() { gate.setPosition(GATE_ACTIVATION_POS); }
    public void deactivate() { gate.setPosition(GATE_REST_POS); }
    public boolean isActivated() { return gate.getPosition() == GATE_ACTIVATION_POS; }
}
