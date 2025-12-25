package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotConstants;

public class Flickers {
    private Servo leftFlicker, rightFlicker;
    private HardwareMap hwmap;
    public Flickers(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    public void init(){
        leftFlicker = hwmap.servo.get(RobotConstants.LEFT_FLICKER_NAME);
        leftFlicker.setDirection(Servo.Direction.REVERSE);
        leftFlicker.setPosition(0);
        rightFlicker = hwmap.servo.get(RobotConstants.RIGHT_FLICKER_NAME);
        rightFlicker.setDirection(Servo.Direction.FORWARD);
        rightFlicker.setPosition(0);
    }

    public void leftFlick(boolean flick){
        leftFlicker.setPosition(flick ? 1 : 0);
    }
    public void rightFlick(boolean flick){
        rightFlicker.setPosition(flick ? 1 : 0);
    }
    public void setLeftFlickerPos(double pwm) {
        leftFlicker.setPosition(pwm);
    }
    public void setRightFlickerPos(double pwm) {
        rightFlicker.setPosition(pwm);
    }
}
