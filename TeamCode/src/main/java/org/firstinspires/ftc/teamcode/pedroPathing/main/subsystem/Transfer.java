package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    public enum TransferState {
        STOP,
        SHOOT,
        COLLECT
    }

    private final Intake intake;
    private final Gate gate;
    private final ColorSensors colorSensors;
    private TransferState currentState = TransferState.STOP;

    public Transfer(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        gate = new Gate();
        colorSensors = new ColorSensors();
    }

    public void init(HardwareMap hardwareMap) {
        intake.init();
        gate.init(hardwareMap);
        colorSensors.init(hardwareMap);
        setState(TransferState.STOP);
        applyState();
    }

    public void setState(TransferState newState) {
        currentState = newState;
    }

    public TransferState getState() {
        return currentState;
    }

    public void stop() {
        setState(TransferState.STOP);
    }

    public void collect() {
        setState(TransferState.COLLECT);
    }

    public void shoot() {
        setState(TransferState.SHOOT);
    }

    public void update() {
        colorSensors.update();

        if (currentState == TransferState.COLLECT
                && (colorSensors.isFull() || intake.isOverCurrentForInterval(2))) {
            currentState = TransferState.STOP;
        }

        applyState();
    }

    private void applyState() {
        switch (currentState) {
            case STOP:
                gate.activate();
                intake.setCurrentState(Intake.IntakeState.STOP);
                break;
            case SHOOT:
                gate.deactivate();
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                break;
            case COLLECT:
                gate.activate();
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                break;
        }

        intake.update();
    }

    public boolean isFull() {
        return colorSensors.isFull();
    }
    public boolean isEmpty() {
        return colorSensors.isEmpty();
    }

    public boolean isOverCurrent() {
        return intake.isOverCurrent();
    }

    public boolean is1Detected() {
        return colorSensors.is1Detected();
    }

    public boolean is2Detected() {
        return colorSensors.is2Detected();
    }

    public boolean is3Detected() {
        return colorSensors.is3Detected();
    }

    public double getFullTime() {
        return colorSensors.getFullTIme();
    }

    public double getCurrent() {
        return intake.getCurrent();
    }

    public Intake getIntake() {
        return intake;
    }

    public Gate getGate() {
        return gate;
    }

    public ColorSensors getColorSensors() {
        return colorSensors;
    }
}
