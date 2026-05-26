package org.firstinspires.ftc.teamcode.pedroPathing.main.constants;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorMode;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorUse;

/**
 * Compatibility builders for debugger OpModes that still exercise MotorConfig.
 *
 * <p>Runtime subsystems should use RobotConstants plus MetaMotor/facade classes
 * directly. Keep new subsystem code out of this class.
 */
@SuppressWarnings("deprecation")
public final class LegacyMotorConfigs {
    private LegacyMotorConfigs() {}

    public static MotorConfig leftFront() {
        return new MotorConfig(
                RobotConstants.LEFT_FRONT_MOTOR_NAME,
                RobotConstants.DRIVETRAIN_MOTOR_TYPE,
                RobotConstants.LEFT_FRONT_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.DRIVETRAIN);
    }

    public static MotorConfig leftBack() {
        return new MotorConfig(
                RobotConstants.LEFT_BACK_MOTOR_NAME,
                RobotConstants.DRIVETRAIN_MOTOR_TYPE,
                RobotConstants.LEFT_BACK_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.DRIVETRAIN);
    }

    public static MotorConfig rightBack() {
        return new MotorConfig(
                RobotConstants.RIGHT_BACK_MOTOR_NAME,
                RobotConstants.DRIVETRAIN_MOTOR_TYPE,
                RobotConstants.RIGHT_BACK_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.DRIVETRAIN);
    }

    public static MotorConfig rightFront() {
        return new MotorConfig(
                RobotConstants.RIGHT_FRONT_MOTOR_NAME,
                RobotConstants.DRIVETRAIN_MOTOR_TYPE,
                RobotConstants.RIGHT_FRONT_MOTOR_DIRECTION,
                RobotConstants.DRIVETRAIN_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.DRIVETRAIN);
    }

    public static MotorConfig intake() {
        return new MotorConfig(
                RobotConstants.INTAKE_MOTOR_NAME,
                RobotConstants.INTAKE_MOTOR_TYPE,
                RobotConstants.INTAKE_MOTOR_DIRECTION,
                RobotConstants.INTAKE_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.FREE_SPIN)
                .setMotorMode(MotorMode.OPEN_LOOP);
    }

    public static MotorConfig shooter() {
        return new MotorConfig(
                RobotConstants.SHOOTER_MOTOR_NAME,
                RobotConstants.SHOOTER_MOTOR_TYPE,
                RobotConstants.SHOOTER_MOTOR_DIRECTION,
                RobotConstants.SHOOTER_ZERO_POWER_BEHAVIOR
        ).setPIDFCoefficients(
                RobotConstants.SHOOTER_VELOCITY_PIDF.kp(),
                RobotConstants.SHOOTER_VELOCITY_PIDF.ki(),
                RobotConstants.SHOOTER_VELOCITY_PIDF.kd(),
                RobotConstants.SHOOTER_VELOCITY_PIDF.ks(),
                RobotConstants.SHOOTER_VELOCITY_PIDF.kv(),
                RobotConstants.SHOOTER_VELOCITY_PIDF.ka()
        ).setMotorMode(MotorMode.VELOCITY_CONTROL);
    }

    public static MotorConfig shooterFollower() {
        return new MotorConfig(
                RobotConstants.SHOOTER_FOLLOWER_MOTOR_NAME,
                RobotConstants.SHOOTER_FOLLOWER_MOTOR_TYPE,
                RobotConstants.SHOOTER_FOLLOWER_DIRECTION,
                RobotConstants.SHOOTER_FOLLOWER_ZERO_POWER_BEHAVIOR
        );
    }

    public static MotorConfig turret() {
        return new MotorConfig(
                RobotConstants.TURRET_MOTOR_NAME,
                RobotConstants.TURRET_MOTOR_TYPE,
                RobotConstants.TURRET_MOTOR_DIRECTION,
                RobotConstants.TURRET_ZERO_POWER_BEHAVIOR
        ).addExternalGearRatio(RobotConstants.TURRET_EXTERNAL_GEAR_RATIO)
                .setMotorUse(MotorUse.MECHANICAL_STOP)
                .setMotorMode(MotorMode.PROFILED_PIDF)
                .setMotionProfileCoefficients(
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxVelocity(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxAcceleration(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxDeceleration(),
                        RobotConstants.TURRET_LIMITS.getMaxPower()
                )
                .setPIDFCoefficients(
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kp(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ki(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kd(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ks(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kv(),
                        RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ka()
                )
                .setMinAngleRadians(RobotConstants.TURRET_MIN_ANGLE_RADIANS)
                .setMaxAngleRadians(RobotConstants.TURRET_MAX_ANGLE_RADIANS);
    }

    public static MotorConfig hang() {
        return new MotorConfig(
                RobotConstants.HANG_MOTOR_NAME,
                RobotConstants.HANG_MOTOR_TYPE,
                RobotConstants.HANG_MOTOR_DIRECTION,
                RobotConstants.HANG_ZERO_POWER_BEHAVIOR
        ).setMotorUse(MotorUse.MECHANICAL_STOP)
                .setMotorMode(MotorMode.SIMPLE_POSITION);
    }
}
