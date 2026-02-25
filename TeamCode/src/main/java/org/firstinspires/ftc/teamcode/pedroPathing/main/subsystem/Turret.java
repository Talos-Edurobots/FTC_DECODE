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
    static double maxPower = .5, kp=0.005, kd = .001, ki=0, ks=0, manualMaxPower = .1, ramp = 1;
    HardwareMap hwmap;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    MotorConfig turret = RobotConstants.TURRET_CONFIG;
    final Pose RED_GOAL_POSE = new Pose(140, 140);
    final Pose BLUE_GOAL_POSE = new Pose(140, 0);
    public Turret(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    double angleToGoal;
    public void faceForward() {
        turret.setPositionInRadians(0);
    }
    public void setAngleRadians(double angleRadians) {
        turret.setPositionInRadians(angleRadians);
        turret.setMotorMode(MotorMode.PROFILED_PIDF);
    }
    public double getAngleToGoal(){
        return angleToGoal;
    }
    public void lookToGoal(Pose pose, boolean isRed) {

        if (isRed) {
            angleToGoal = Math.atan2(RED_GOAL_POSE.getY()-pose.getY(), RED_GOAL_POSE.getX()-pose.getX());
        } else {
           angleToGoal = Math.atan2(BLUE_GOAL_POSE.getY()-pose.getY(), BLUE_GOAL_POSE.getX()-pose.getX());
        }

        // TODO: add robot heading to angleToGoal
        double robotAngle = (pose.getHeadingAsUnitVector().getTheta() + Math.PI/2) % (2*Math.PI);
        setAngleRadians(angleToGoal - robotAngle);
        turret.setMotorMode(MotorMode.PROFILED_PIDF);
        telemetryM.addData("angle to goal", angleToGoal - robotAngle);
        telemetryM.addData("robot angle", robotAngle);
    }
    public void init() {
        turret.init(hwmap);
        faceForward();
    }
    public void limelightAim(LLResult result) {
        turret.maxPower = maxPower;
        turret.kP = kp; turret.kD = kd; turret.kI = ki; turret.kS = ks;
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
    public void manualControl(double input) {
        turret.extraPower = manualMaxPower * input;
        telemetryM.addData("turret extra power", turret.extraPower);
    }
    public void loop() {
        turret.maxPower = maxPower;
        turret.update();
        telemetryM.addData("turret mode", turret.getMotorMode());
        telemetryM.addData("power", turret.getPower());
        telemetryM.addData("velocity", turret.getVelocity());
        telemetryM.addData("ref vel", turret.getvRef());
        telemetryM.addData("position", turret.getCurrentPosition());
        telemetryM.addData("ref pos", turret.getxRef());
        telemetryM.addData("ref a", turret.getaRef());
        telemetryM.addData("current", turret.getCurrent());
        telemetryM.addData("ks motor", turret.kS);
        telemetryM.addData("target", turret.getTargetPositionTicks());
    }
}
