package org.firstinspires.ftc.teamcode.pedroPathing.main.constants;

import com.bylazar.field.Style;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.DcMotorConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorMode;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorUse;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;

public final class RobotConstants {
    private RobotConstants(){}

    /* Robot Configuration Constants */
// Drivetrain motors
    public static MotorConfig LEFT_FRONT_CONFIG = new MotorConfig(
            "leftFront",
            GoBILDAMotorTypes.MOTOR_312_RPM,
            DcMotorSimple.Direction.REVERSE,
            DcMotor.ZeroPowerBehavior.BRAKE
    ).setMotorUse(MotorUse.DRIVETRAIN);
    public static MotorConfig LEFT_BACK_CONFIG = new MotorConfig(
            "leftBack",
            GoBILDAMotorTypes.MOTOR_312_RPM,
            DcMotorSimple.Direction.REVERSE,
            DcMotor.ZeroPowerBehavior.BRAKE
    ).setMotorUse(MotorUse.DRIVETRAIN);
    public static MotorConfig RIGHT_BACK_CONFIG = new MotorConfig(
            "rightBack",
            GoBILDAMotorTypes.MOTOR_312_RPM,
            DcMotorSimple.Direction.FORWARD,
            DcMotor.ZeroPowerBehavior.BRAKE
    ).setMotorUse(MotorUse.DRIVETRAIN);
    public static MotorConfig RIGHT_FRONT_CONFIG = new MotorConfig(
            "rightFront",
            GoBILDAMotorTypes.MOTOR_312_RPM,
            DcMotorSimple.Direction.FORWARD,
            DcMotor.ZeroPowerBehavior.BRAKE
    ).setMotorUse(MotorUse.DRIVETRAIN);
    public static MotorConfig INTAKE_CONFIG = new MotorConfig(
            "intake",
            GoBILDAMotorTypes.MOTOR_1150_RPM,
            DcMotorSimple.Direction.FORWARD
    ).setMotorUse(MotorUse.FREE_SPIN)
            .setMotorMode(MotorMode.OPEN_LOOP);

    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final GoBILDAMotorTypes SHOOTER_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_6000_RPM;
    public static final DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final PIDFFCoefficients SHOOTER_VELOCITY_PIDF =
            new PIDFFCoefficients(.005, 0, 0, .02, 0.0052684109772247485, 0);
    public static final MotorLimits SHOOTER_LIMITS = MotorLimits.defaults();

    public static MotorConfig SHOOTER_CONFIG = new MotorConfig(
            SHOOTER_MOTOR_NAME,
            SHOOTER_MOTOR_TYPE,
            SHOOTER_MOTOR_DIRECTION
    ).setPIDFCoefficients(
            SHOOTER_VELOCITY_PIDF.kp(),
            SHOOTER_VELOCITY_PIDF.ki(),
            SHOOTER_VELOCITY_PIDF.kd(),
            SHOOTER_VELOCITY_PIDF.ks(),
            SHOOTER_VELOCITY_PIDF.kv(),
            SHOOTER_VELOCITY_PIDF.ka()
    )
            .setMotorMode(MotorMode.VELOCITY_CONTROL);
    public static MotorConfig TURRET_CONFIG = new MotorConfig(
            "turret",
            GoBILDAMotorTypes.MOTOR_312_RPM,
            DcMotorSimple.Direction.FORWARD,
            DcMotor.ZeroPowerBehavior.FLOAT
    ).addExternalGearRatio((double) 2.8 /1)
            .setMotorUse(MotorUse.MECHANICAL_STOP)
            .setMotorMode(MotorMode.PROFILED_PIDF)
            .setMotionProfileCoefficients(1500, 4000, 1)
            .setPIDFCoefficients(0.01, 0.000001, 0.002, 0.18, 0.005687094208999908, 0.0002)
//           .setPIDFCoefficients(.005, 0, 0.001, 0, 0, 0)
            .setMinAngleRadians(Math.toRadians(-160))
            .setMaxAngleRadians(Math.toRadians(160));
    public static MotorConfig HANG_CONFIG = new MotorConfig(
            "hang",
            GoBILDAMotorTypes.MOTOR_117_RPM,
            DcMotorSimple.Direction.FORWARD,
            DcMotor.ZeroPowerBehavior.BRAKE
    ).setMotorUse(MotorUse.MECHANICAL_STOP)
            .setMotorMode(MotorMode.SIMPLE_POSITION);

    public static String SHOOTER_LED_RED  = "shooterRed";
    public static String SHOOTER_LED_GREEN= "shooterGreen";
    // Pinpoint
    public static String RIGHT_SERVO_NAME = "rightServo";
    public static String LEFT_SERVO_NAME = "leftServo";
    public static String LEFT_FLICKER_NAME = "leftFlicker";
    public static String RIGHT_FLICKER_NAME = "rightFlicker";
    public static String PINPOINT_NAME    = "pinpoint";
    public static String LED_RIGHT = "led_right";
    public static String LED_LEFT = "led_left";
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIRECTION  = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static double PINPOINT_FORWARD_POD_Y = 3.4; // in inches
    public static double PINPOINT_STRAFE_POD_X = 4; // in inches

    public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
    );

    /* PID, Velocity Constants */
    public static double DrivetrainMaxAcceleration = 5; // motor power / second


    public static Style ROBOT_DRAW_STYLE = new Style(
            "", "#3F51B5", .75
    );
    public static final String ALLIANCE_KEY = "Alliance";
    public static final String FOLLOWER_KEY = "Follower";
}
