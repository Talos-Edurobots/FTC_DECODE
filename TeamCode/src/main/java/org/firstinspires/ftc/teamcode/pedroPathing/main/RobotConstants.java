package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.field.Style;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;

public final class RobotConstants {
    private RobotConstants(){}

    /* Robot Configuration Constants */
    // Drivetrain motors
    public static String LEFT_FRONT_NAME  = "leftFront";
    public static DcMotorSimple.Direction LEFT_FRONT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static String RIGHT_BACK_NAME  = "rightBack";
    public static DcMotorSimple.Direction RIGHT_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    public static String LEFT_BACK_NAME   = "leftBack";
    public static DcMotorSimple.Direction LEFT_BACK_DIRECTION   = DcMotorSimple.Direction.REVERSE;
    // Intake
    public static String INTAKE_NAME      = "intake";
    // Shooter
    public static DcMotorSimple.Direction INTAKE_DIRECTION      = DcMotorSimple.Direction.FORWARD;
    public static String SHOOTER_NAME     = "shooter";
    // Pinpoint
    public static DcMotorSimple.Direction SHOOTER_DIRECTION     = DcMotorSimple.Direction.REVERSE;
    public static String RIGHT_SERVO_NAME = "rightServo";
    public static String LEFT_SERVO_NAME = "leftServo";
    public static String LEFT_FLICKER_NAME = "leftFlicker";
    public static String RIGHT_FLICKER_NAME = "rightFlicker";
    public static String PINPOINT_NAME    = "pinpoint";
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIRECTION  = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static double PINPOINT_FORWARD_POD_Y = 3.4; // in inches
    public static double PINPOINT_STRAFE_POD_X = 4; // in inches

    public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
    );

    /* PID, Velocity Constants */
    public static double DrivetrainMaxAcceleration = 8; // motor power / second

    public static double DrivetrainMotorTicksPerRevolution = DcMotorConstants.Motor312EncoderResolution;
    public static double ShooterMotorTicksPerRevolution = DcMotorConstants.Motor6000EncoderResolution;
    public static double IntakeMotorTicksPerRevolution = DcMotorConstants.Motor1150EncoderResolution;

    public static double INTAKE_MAX_VELOCITY = 2700; // ticks per second

    public static Style ROBOT_DRAW_STYLE = new Style(
            "", "#3F51B5", .75
    );
    public static double SHOOTER_K_P = 0.01;
    public static double SHOOTER_K_I = 0;
    public static double SHOOTER_K_D = 0;
    public static double SHOOTER_K_S = 0.02;
    public static double SHOOTER_K_V = 0.0004;
}
