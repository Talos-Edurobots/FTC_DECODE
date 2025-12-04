package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotConstants;

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
    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    private DcMotorEx intakeMotor;
    public Intake(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    public void init() {
        intakeMotor = hwMap.get(DcMotorEx.class, RobotConstants.INTAKE_NAME);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(RobotConstants.INTAKE_DIRECTION);
    }
    public void update(){
        switch (currentState){
            case INTAKE:
                intakeMotor.setVelocity(RobotConstants.INTAKE_MAX_VELOCITY * .9);
                break;
            case OUTTAKE:
                intakeMotor.setPower(-1 * RobotConstants.INTAKE_MAX_VELOCITY * .9);
                break;
            case STOP:
                intakeMotor.setPower(0);
                break;
        }
    }
}
