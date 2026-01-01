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
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private double externalGearRatio = 1.0;
    private DcMotorEx motor;
    private double kP, kI, kD, kS, kV, kA;

    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotor.Direction direction
    ) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.direction = direction;
    }
    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType
    ) {
        this(hardwareName, motorType, DcMotor.Direction.FORWARD);
    }
    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotor.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        this(hardwareName, motorType, direction);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }
    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
            DcMotor.RunMode runMode
    ) {
        this(hardwareName, motorType, direction);
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
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
    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public MotorConfig setPIDFCoefficients(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }
}
