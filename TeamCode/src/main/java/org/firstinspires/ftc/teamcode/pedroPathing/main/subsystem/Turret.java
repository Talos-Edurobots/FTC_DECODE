package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;

public class Turret {
    HardwareMap hwmap;
    MotorConfig turret = RobotConstants.TURRET_CONFIG;
    final Pose RED_GOAL_POSE = new Pose(144, 144);
    final Pose BLUE_GOAL_POSE = new Pose(144, -144);
    public Turret(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    public void faceForward() {
        turret.setPositionInRadians(0);
    }
    public void setAngleRadians(double angleRadians) {
        turret.setPositionInRadians(angleRadians);
    }
    public void lookToGoal(Pose pose, boolean isRed) {
        double angleToGoal;
        if (isRed) {
            angleToGoal = Math.atan2(RED_GOAL_POSE.getX()-pose.getX(), RED_GOAL_POSE.getY()-pose.getY());
        } else {
           angleToGoal = Math.atan2(BLUE_GOAL_POSE.getX()-pose.getX(), BLUE_GOAL_POSE.getY()-pose.getY());
        }
        setAngleRadians(angleToGoal);
    }
    public void init() {
        turret.init(hwmap);
        faceForward();
    }
    public void loop(double dt, double batteryVoltage) {
        turret.updatePositionProfiledPIDF(dt, batteryVoltage);
    }
}
