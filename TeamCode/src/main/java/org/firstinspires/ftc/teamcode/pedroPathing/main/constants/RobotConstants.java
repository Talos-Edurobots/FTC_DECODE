package org.firstinspires.ftc.teamcode.pedroPathing.main.constants;

import com.bylazar.field.Style;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;

public final class RobotConstants {
    private RobotConstants(){}

    /* Robot Configuration Constants */
// Drivetrain motors
    public static final String LEFT_FRONT_MOTOR_NAME = "leftFront";
    public static final String LEFT_BACK_MOTOR_NAME = "leftBack";
    public static final String RIGHT_BACK_MOTOR_NAME = "rightBack";
    public static final String RIGHT_FRONT_MOTOR_NAME = "rightFront";
    public static final GoBILDAMotorTypes DRIVETRAIN_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_312_RPM;
    public static final DcMotorSimple.Direction LEFT_FRONT_MOTOR_DIRECTION =
            DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_BACK_MOTOR_DIRECTION =
            DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_BACK_MOTOR_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_FRONT_MOTOR_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior DRIVETRAIN_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;
    public static final MotorLimits DRIVETRAIN_LIMITS = MotorLimits.defaults();

    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final GoBILDAMotorTypes INTAKE_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_1150_RPM;
    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;
    public static final MotorLimits INTAKE_LIMITS = new MotorLimits(1.0, 7.0);

    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final GoBILDAMotorTypes SHOOTER_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_6000_RPM;
    public static final DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotor.ZeroPowerBehavior SHOOTER_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;
    public static final PIDFFCoefficients SHOOTER_VELOCITY_PIDF =
            new PIDFFCoefficients(.005, 0, 0, .02, 0.0052684109772247485, 0);
    public static final MotorLimits SHOOTER_LIMITS = MotorLimits.defaults();
    public static final String SHOOTER_FOLLOWER_MOTOR_NAME = "shooter2";
    public static final GoBILDAMotorTypes SHOOTER_FOLLOWER_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_6000_RPM;
    public static final DcMotorSimple.Direction SHOOTER_FOLLOWER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior SHOOTER_FOLLOWER_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;

    public static final String TURRET_MOTOR_NAME = "turret";
    public static final GoBILDAMotorTypes TURRET_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_312_RPM;
    public static final DcMotorSimple.Direction TURRET_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior TURRET_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;
    public static final double TURRET_EXTERNAL_GEAR_RATIO = 2.8;
    public static final double TURRET_HARD_STOP_START_TICKS = -367.0;
    public static final double TURRET_ZERO_OFFSET_TICKS = -TURRET_HARD_STOP_START_TICKS;
    public static final double TURRET_ZERO_OFFSET_MOTOR_RADIANS =
            TURRET_ZERO_OFFSET_TICKS / TURRET_MOTOR_TYPE.getTicksPerRadian();
    public static final double TURRET_ZERO_OFFSET_MECHANISM_RADIANS =
            TURRET_ZERO_OFFSET_MOTOR_RADIANS / TURRET_EXTERNAL_GEAR_RATIO;
    public static final MotionProfilingCoefficients TURRET_PROFILE_COEFFICIENTS =
            new MotionProfilingCoefficients(
                    new PIDFFCoefficients(0.068, 0, 0.002, 1.2, 0.005603855012349794, 0.0004),
                    1800,
                    4500,
                    4500
            );
    public static final MotorLimits TURRET_LIMITS = new MotorLimits(1.0, Double.POSITIVE_INFINITY);
    public static final double TURRET_MIN_ANGLE_RADIANS = -TURRET_ZERO_OFFSET_MECHANISM_RADIANS;
    public static final double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(120);
    public static final MotionProfilingCoefficients TURRET_CONFIGURABLE_PROFILE_DEFAULTS =
            new MotionProfilingCoefficients(
                    new PIDFFCoefficients(0.005, 0, 0, 0.3, 0.005687094208999908, 0.0004),
                    1800,
                    3000,
                    2000
            );

    public static final String HANG_MOTOR_NAME = "hang";
    public static final GoBILDAMotorTypes HANG_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_117_RPM;
    public static final DcMotorSimple.Direction HANG_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior HANG_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    public static String SHOOTER_LED_RED  = "shooterRed";
    public static String SHOOTER_LED_GREEN= "shooterGreen";
    // Pinpoint
    public static String RIGHT_SERVO_NAME = "rightServo";
    public static String LEFT_SERVO_NAME = "leftServo";
    public static String RIGHT_FLICKER_NAME = "rightFlicker";
    public static String LEFT_FLICKER_NAME = "leftFlicker";
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
