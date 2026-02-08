package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorMode;

@Configurable
public class Turret {
    static double maxPower = .2, kp=0.005, kd = .001;
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
    public void limelightAim(LLResult result) {
        turret.maxPower = maxPower;
        turret.kP = kp; turret.kD = kd;
        if (result != null) {
            if (result.isValid()) {
                telemetryM.addLine("valid result");
                if (result.getTx() == 0) {
                    turret.manualPositionPIDF(0);
                    telemetryM.addLine("0 power");
                }
                else {
                    turret.manualPositionPIDF(-result.getTx());
                    telemetryM.addLine("running turret");
                }
            }
            else {
                turret.manualPositionPIDF(0);
                telemetryM.addLine("invalid result");
            }
        }
        else {
            telemetryM.addLine("null result");
        }
        telemetryM.addData("kp", turret.kP);
        telemetryM.addData("kd", turret.kD);
        telemetryM.addData("tx", result.getTx());
        telemetryM.addData("max power", turret.maxPower);
        telemetryM.addData("power", turret.getPower());
    }
    public void loop() {
        turret.update();
        telemetryM.addData("turret mode", turret.getMotorMode());
    }
}
