package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;

public class Leds {
    Servo left, right;
    public void init(HardwareMap hwmap) {
        left = hwmap.servo.get(RobotConstants.LED_LEFT);
        right = hwmap.servo.get(RobotConstants.LED_RIGHT);
    }
    public void setLeft(double val) {
        left.setPosition(val);
    }
    public void setRight(double val) {
        right.setPosition(val);
    }
}
