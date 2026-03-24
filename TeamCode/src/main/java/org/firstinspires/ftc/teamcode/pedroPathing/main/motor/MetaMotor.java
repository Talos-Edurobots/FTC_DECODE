package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;

public class MetaMotor {
    private GoBILDAMotorTypes motorType;
    private String hwName;
    private HardwareMap hwMap;
    private DcMotorSimple.Direction direction;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private MotorController controller;
    public MetaMotor() {}
}
