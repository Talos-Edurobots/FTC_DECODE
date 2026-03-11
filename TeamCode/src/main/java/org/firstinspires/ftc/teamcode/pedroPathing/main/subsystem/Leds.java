package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;

@Configurable
public class Leds {
    Servo left, right;
    double timeLeft, timeRight;
    boolean leftUse1, rightUse1;
    static double speed = 1;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
    public void blinkLeft(double time, double dt, double color1, double color2) {
        timeLeft += dt;
        if (timeLeft > time && leftUse1) {
            timeLeft -= time;
            leftUse1 = false;
            left.setPosition(color2);
        }
        else if (timeLeft > time && !leftUse1) {
            leftUse1 = true;
            timeLeft-=time;
            left.setPosition(color1);
        }
        telemetryM.addData("led time", timeLeft);
        telemetryM.addData("led dt", dt);
        telemetryM.addData("use 1", leftUse1);
    }
    public void blinkRight(double time, double dt, double color1, double color2) {
        timeRight += dt;
        if (timeRight > time && rightUse1) {
            rightUse1 = false;
            timeRight -= time;
            right.setPosition(color2);
        }
        else if (timeRight > time && !rightUse1) {
            rightUse1 = true;
            timeRight -= time;
            right.setPosition(color1);
        }
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
