package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;

public class Flickers {
    private Servo leftFlicker, rightFlicker;
    private HardwareMap hwmap;
    private final double LEFT_FLICKER_FLICK_POS = 1;
    private final double LEFT_FLICKER_REST_POS = 0.5;
    private final double RIGHT_FLICKER_FLICK_POS = 1;
    private final double RIGHT_FLICKER_REST_POS = 0.5;
    public void init(HardwareMap hwmap){
        this.hwmap = hwmap;
        leftFlicker = hwmap.servo.get(RobotConstants.LEFT_FLICKER_NAME);
        leftFlicker.setDirection(Servo.Direction.REVERSE);
        leftFlicker.setPosition(LEFT_FLICKER_REST_POS);
        rightFlicker = hwmap.servo.get(RobotConstants.RIGHT_FLICKER_NAME);
        rightFlicker.setDirection(Servo.Direction.FORWARD);
        rightFlicker.setPosition(RIGHT_FLICKER_REST_POS);
    }

    public void leftFlick(boolean flick){
        leftFlicker.setPosition(flick ? LEFT_FLICKER_FLICK_POS : LEFT_FLICKER_REST_POS);
    }
    public void rightFlick(boolean flick){
        rightFlicker.setPosition(flick ? RIGHT_FLICKER_FLICK_POS : RIGHT_FLICKER_REST_POS);
    }
    public void setLeftFlickerPos(double pwm) {
        leftFlicker.setPosition(pwm);
    }
    public void setRightFlickerPos(double pwm) {
        rightFlicker.setPosition(pwm);
    }
}
