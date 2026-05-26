package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.OpenLoopMotor;

public class DriveTrain {
    HardwareMap hwMap;
    OpenLoopMotor lf, rf, lb, rb;
    //    Follower follower = new Follower(FollowerConstants.class);
    public DriveTrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }
    public void init(){
        lf = new OpenLoopMotor(
                RobotConstants.LEFT_FRONT_MOTOR_NAME,
                RobotConstants.LEFT_FRONT_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR,
                RobotConstants.DRIVETRAIN_LIMITS
        );
        rf = new OpenLoopMotor(
                RobotConstants.RIGHT_FRONT_MOTOR_NAME,
                RobotConstants.RIGHT_FRONT_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR,
                RobotConstants.DRIVETRAIN_LIMITS
        );
        lb = new OpenLoopMotor(
                RobotConstants.LEFT_BACK_MOTOR_NAME,
                RobotConstants.LEFT_BACK_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR,
                RobotConstants.DRIVETRAIN_LIMITS
        );
        rb = new OpenLoopMotor(
                RobotConstants.RIGHT_BACK_MOTOR_NAME,
                RobotConstants.RIGHT_BACK_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR,
                RobotConstants.DRIVETRAIN_LIMITS
        );
        lf.init(hwMap);
        rf.init(hwMap);
        lb.init(hwMap);
        rb.init(hwMap);
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
        if (target < current) return target;
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
