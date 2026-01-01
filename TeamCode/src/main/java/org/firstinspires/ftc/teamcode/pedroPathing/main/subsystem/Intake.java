package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;

public class Intake {
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOP
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
    }
    public void update(){
        switch (currentState){
            case INTAKE:
                motor.setPower(RobotConstants.INTAKE_MAX_VELOCITY * .9);
                break;
            case OUTTAKE:
                motor.setPower(-1 * RobotConstants.INTAKE_MAX_VELOCITY * .9);
                break;
            case STOP:
                motor.setPower(0);
                break;
        }
    }
    public double getVelocity(){
        return motor.getVelocity();
    }
    public double getCurrent(){
        return motor.getCurrent();
    }
}
