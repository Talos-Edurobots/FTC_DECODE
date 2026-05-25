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
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
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
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final GoBILDAMotorTypes INTAKE_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_1150_RPM;
    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;
    public static final MotorLimits INTAKE_LIMITS = new MotorLimits(1.0, 7.0);
    public static MotorConfig INTAKE_CONFIG = new MotorConfig(
            INTAKE_MOTOR_NAME,
            INTAKE_MOTOR_TYPE,
            INTAKE_MOTOR_DIRECTION
    ).setMotorUse(MotorUse.FREE_SPIN)
            .setMotorMode(MotorMode.OPEN_LOOP);

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
    public static MotorConfig SHOOTER2_CONFIG = new MotorConfig(
            SHOOTER_FOLLOWER_MOTOR_NAME,
            SHOOTER_FOLLOWER_MOTOR_TYPE,
            SHOOTER_FOLLOWER_DIRECTION
    );
    public static final MotorConfig[] SHOOTER_MOTOR_CONFIGS = new MotorConfig[]{
            SHOOTER_CONFIG,
            SHOOTER2_CONFIG
    };
    public static final String TURRET_MOTOR_NAME = "turret";
    public static final GoBILDAMotorTypes TURRET_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_312_RPM;
    public static final DcMotorSimple.Direction TURRET_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior TURRET_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;
    public static final double TURRET_EXTERNAL_GEAR_RATIO = 2.8;
    public static final MotionProfilingCoefficients TURRET_PROFILE_COEFFICIENTS =
            new MotionProfilingCoefficients(
                    new PIDFFCoefficients(0.068, 0, 0.002, 1.2, 0.005687094208999908, 0.0004),
                    1800,
                    4500,
                    4500
            );
    public static final MotorLimits TURRET_LIMITS = new MotorLimits(1.0, Double.POSITIVE_INFINITY);
    public static final double TURRET_MIN_ANGLE_RADIANS = Math.toRadians(-100);
    public static final double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(120);
    public static MotorConfig TURRET_CONFIG = new MotorConfig(
            TURRET_MOTOR_NAME,
            TURRET_MOTOR_TYPE,
            TURRET_MOTOR_DIRECTION,
            TURRET_ZERO_POWER_BEHAVIOR
    ).addExternalGearRatio(TURRET_EXTERNAL_GEAR_RATIO)
            .setMotorUse(MotorUse.MECHANICAL_STOP)
            .setMotorMode(MotorMode.PROFILED_PIDF)
            .setMotionProfileCoefficients(1800, 3000, 2000, 1)
            .setPIDFCoefficients(0.005, 0, 0, 0.3, 0.005687094208999908, 0.0004)
//           .setPIDFCoefficients(.005, 0, 0.001, 0, 0, 0)
            .setMinAngleRadians(TURRET_MIN_ANGLE_RADIANS)
            .setMaxAngleRadians(TURRET_MAX_ANGLE_RADIANS);
    public static final String HANG_MOTOR_NAME = "hang";
    public static final GoBILDAMotorTypes HANG_MOTOR_TYPE = GoBILDAMotorTypes.MOTOR_117_RPM;
    public static final DcMotorSimple.Direction HANG_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior HANG_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;
    public static MotorConfig HANG_CONFIG = new MotorConfig(
            HANG_MOTOR_NAME,
            HANG_MOTOR_TYPE,
            HANG_MOTOR_DIRECTION,
            HANG_ZERO_POWER_BEHAVIOR
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
