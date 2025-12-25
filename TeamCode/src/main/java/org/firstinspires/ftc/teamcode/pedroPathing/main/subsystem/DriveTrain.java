package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotConstants;

public class DriveTrain {
    HardwareMap hwMap;
    DcMotor lf, rf, lb, rb;
    Follower follower = new Follower(FollowerConstants.class);
    public DriveTrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }
    public void init(){
        lf = hwMap.dcMotor.get(RobotConstants.LEFT_FRONT_NAME);
        rf = hwMap.dcMotor.get(RobotConstants.RIGHT_FRONT_NAME);
        lb = hwMap.dcMotor.get(RobotConstants.LEFT_BACK_NAME);
        rb = hwMap.dcMotor.get(RobotConstants.RIGHT_BACK_NAME);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fieldCentricDrive(double x, double y, double rx, double botHeading, double speed){
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = speed*((rotY + rotX + rx) / denominator);
        double backLeftPower = speed*((rotY - rotX + rx) / denominator);
        double frontRightPower = speed*((rotY - rotX - rx) / denominator);
        double backRightPower = speed*(rotY + rotX - rx) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }

    private double rampPower(double current, double target, double dt) {
        double maxPowerChange = RobotConstants.DrivetrainMaxAcceleration * dt;
        double diff = target - current;
        if (Math.abs(diff) > maxPowerChange) {
            diff = Math.signum(diff) * maxPowerChange;
        }
        return current + diff;
    }

    public void FieldCentricAccelerationDrive(double x, double y, double rx, double botHeading, double speed, double dt){
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double targetFrontLeftPower = speed*((rotY + rotX + rx) / denominator);
        double targetBackLeftPower = speed*((rotY - rotX + rx) / denominator);
        double targetFrontRightPower = speed*((rotY - rotX - rx) / denominator);
        double targetBackRightPower = speed*(rotY + rotX - rx) / denominator;

        double frontLeftPower = rampPower(lf.getPower(), targetFrontLeftPower, dt);
        double backLeftPower = rampPower(lb.getPower(), targetBackLeftPower, dt);
        double frontRightPower = rampPower(rf.getPower(), targetFrontRightPower, dt);
        double backRightPower = rampPower(rb.getPower(), targetBackRightPower, dt);

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }

    public void park(){

    }
}
