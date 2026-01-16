package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;

public class Hang {
    MotorConfig motor = RobotConstants.HANG_CONFIG;
    HardwareMap hwmap;
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        motor.init(hwmap);
    }
    public void update(double power, int posInDegrees) {
        motor.updateSimplePositionControl((int) (motor.getMotorType().getTicksPerDegree() * posInDegrees));
    }
}
