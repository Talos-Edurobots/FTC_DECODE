package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;

public class Intake {
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOP,
        KEEP
    }
    private IntakeState currentState = IntakeState.STOP;

    public IntakeState getCurrentState() {
        return currentState;
    }

    public void setCurrentState(IntakeState currentState) {
        this.currentState = currentState;
    }
    HardwareMap hwMap;
    MotorConfig motor = RobotConstants.INTAKE_CONFIG;
    public Intake(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    public void init() {
        motor.init(hwMap);
        motor.setCurrentAlert(6);
    }
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }
    public void update(){
        switch (currentState){
            case INTAKE:
                motor.setPower(1);
                break;
            case OUTTAKE:
                motor.setPower(-1);
                break;
            case STOP:
                motor.setPower(0);
                break;
            case KEEP:
                motor.setPower(.3);
        }
    }
    public double getVelocity(){
        return motor.getVelocity();
    }
    public double getCurrent(){
        return motor.getCurrent();
    }
}
