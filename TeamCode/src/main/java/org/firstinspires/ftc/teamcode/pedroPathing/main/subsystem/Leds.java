package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;

@Configurable
public class Leds {
    Servo left, right;
    static double speed = 1;
    double rgbColor;
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
    public void rgb(double dt) {
        rgbColor += dt * speed;
        if (rgbColor > .72) {
            rgbColor = .28;
        }
        setLeft(rgbColor);
        setRight(rgbColor);
    }
}
