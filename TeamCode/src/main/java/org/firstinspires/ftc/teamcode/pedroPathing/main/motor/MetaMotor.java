package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngleUnit;

public class MetaMotor {
    private DcMotorEx motor;
    private double currentAlert = Double.POSITIVE_INFINITY;
    private Angle maxAngle = Angle.fromRadians(Double.POSITIVE_INFINITY);
    private GoBILDAMotorTypes motorType;
    private String hwName;
    private HardwareMap hwMap;
    private DcMotorSimple.Direction direction;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private MotorController controller;

    public void motorType(GoBILDAMotorTypes motorType) {
        this.motorType = motorType;
    }
    public void hwName(String hwName) {
        this.hwName = hwName;
    }
    public void direction(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }
    public void zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }
    public void controller(MotorController controller) {
        this.controller = controller;
    }
    public MetaMotor() {}
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        motor = hwMap.get(DcMotorEx.class, hwName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
