package org.firstinspires.ftc.teamcode.pedroPathing.main.config;

import com.bylazar.field.Style;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.main.DcMotorConstants;

public final class RobotConstants {
    private RobotConstants(){}

    /* Robot Configuration Constants */
    // Drivetrain motors
    public static MotorConfig LEFT_FRONT_CONFIG = new MotorConfig(
            "leftFront",
            GoBildaMotor.MOTOR_312_RPM,
            DcMotorSimple.Direction.REVERSE
    );
    public static MotorConfig LEFT_BACK_CONFIG = new MotorConfig(
            "leftBack",
            GoBildaMotor.MOTOR_312_RPM,
            DcMotorSimple.Direction.REVERSE
    );
    public static MotorConfig RIGHT_BACK_CONFIG = new MotorConfig(
            "rightBack",
            GoBildaMotor.MOTOR_312_RPM,
            DcMotorSimple.Direction.FORWARD
    );
    public static MotorConfig RIGHT_FRONT_CONFIG = new MotorConfig(
            "rightFront",
            GoBildaMotor.MOTOR_312_RPM,
            DcMotorSimple.Direction.FORWARD
    );
    public static MotorConfig INTAKE_CONFIG = new MotorConfig(
            "intake",
            GoBildaMotor.MOTOR_1150_RPM,
            DcMotorSimple.Direction.FORWARD
    );
    public static MotorConfig SHOOTER_CONFIG = new MotorConfig(
            "shooter",
            GoBildaMotor.MOTOR_6000_RPM,
            DcMotorSimple.Direction.REVERSE
    );
    public static MotorConfig TURRET_CONFIG = new MotorConfig(
            "turret",
            GoBildaMotor.MOTOR_1150_RPM,
            DcMotorSimple.Direction.FORWARD
    ).setExternalGearRatio((double) 130 /270)
    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    public static String SHOOTER_LED_RED  = "shooterRed";
    public static String SHOOTER_LED_GREEN= "shooterGreen";
    // Pinpoint
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
