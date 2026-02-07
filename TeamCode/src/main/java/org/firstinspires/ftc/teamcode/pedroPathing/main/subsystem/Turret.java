package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorMode;

public class Turret {
    HardwareMap hwmap;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        turret.setMotorMode(MotorMode.PROFILED_PIDF);
    }
    public void lookToGoal(Pose pose, boolean isRed) {
        double angleToGoal;
        if (isRed) {
            angleToGoal = Math.atan2(RED_GOAL_POSE.getX()-pose.getX(), RED_GOAL_POSE.getY()-pose.getY());
        } else {
           angleToGoal = Math.atan2(BLUE_GOAL_POSE.getX()-pose.getX(), BLUE_GOAL_POSE.getY()-pose.getY());
        }
        setAngleRadians(angleToGoal);
        turret.setMotorMode(MotorMode.PROFILED_PIDF);
    }
    public void init() {
        turret.init(hwmap);
        faceForward();
    }
    public void limelightAim(double tx) {
        turret.manualPositionPIDF(tx);
        turret.setMotorMode(MotorMode.CUSTOM_ERROR_POSITION_PIDF);
    }
    public void loop() {
        turret.update();
        telemetryM.addData("turret mode", turret.getMotorMode());
    }
}
