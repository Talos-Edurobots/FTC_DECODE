package org.firstinspires.ftc.teamcode.pedroPathing.main.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorConfig {

    private final String hardwareName;
    private final GoBildaMotor motorType;
    private DcMotor.Direction direction;
    private double externalGearRatio = 1.0;
    private DcMotorEx motor;

    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotor.Direction direction
    ) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.direction = direction;
    }

    public String getHardwareName() {
        return hardwareName;
    }

    public GoBildaMotor getMotorType() {
        return motorType;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    /** Initializes and configures the motor from hardwareMap */
    public DcMotorEx init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, hardwareName);
        motor.setDirection(direction);
        return motor;
    }
    public MotorConfig setExternalGearRatio(double externalGearRatio) {
        this.externalGearRatio = externalGearRatio;
        return this;
    }
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }
    public double getVelocity(boolean useExternalGearRatio) {
        return useExternalGearRatio ? getVelocity() * externalGearRatio : getVelocity();
    }
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
    public MotorConfig setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
        return this;
    }
    public MotorConfig setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }
}
