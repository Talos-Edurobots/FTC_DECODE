package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public final class RobotConstants {
    private RobotConstants(){}

    /* Robot Configuration Constants */
    // Drivetrain motors
    public static String LEFT_FRONT_NAME = "leftFront";
    public static  String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static String LEFT_BACK_NAME = "leftBack";
    // Intake
    public static String INTAKE_NAME = "intake";
    // Shooter
    public static String SHOOTER_NAME = "shooter";
    // Pinpoint
    public static String PINPOINT_NAME = "pinpoint";

    public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    /* PID, Velocity Constants */
    public static double DrivetrainMaxAcceleration = 4; // motor power / second


}
