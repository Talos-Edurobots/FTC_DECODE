package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;

/**
 * Shared shooter calibration LUTs and goal-distance helpers.
 */
public final class ShooterHoodLuts {
    public static final double BLUE_GOAL_X = 15.0;
    public static final double BLUE_GOAL_Y = 128.0;

    public static final ShooterVelocityLut SHOOTER_VELOCITY_LUT = new ShooterVelocityLut(
            ShooterVelocityLut.sample(18.0, 1250.0),
            ShooterVelocityLut.sample(36.0, 1325.0),
            ShooterVelocityLut.sample(60.0, 1450.0),
            ShooterVelocityLut.sample(84.0, 1575.0),
            ShooterVelocityLut.sample(108.0, 1700.0)
    );

    public static final HoodAngleLut HOOD_ANGLE_LUT = new HoodAngleLut(
            HoodAngleLut.sample(18.0, 1250.0, 0.10),
            HoodAngleLut.sample(36.0, 1325.0, 0.13),
            HoodAngleLut.sample(60.0, 1450.0, 0.18),
            HoodAngleLut.sample(84.0, 1575.0, 0.24),
            HoodAngleLut.sample(108.0, 1700.0, 0.30)
    );

    private ShooterHoodLuts() {}

    public static double distanceToBlueGoal(Pose robotPose) {
        return ShooterVelocityLut.distanceToGoal(robotPose, BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    public static double distanceToBlueGoal(double robotX, double robotY) {
        return ShooterVelocityLut.distanceToGoal(robotX, robotY, BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    public static double getShooterVelocityForBlueGoal(Pose robotPose) {
        return SHOOTER_VELOCITY_LUT.getTargetVelocity(distanceToBlueGoal(robotPose));
    }

    public static double getHoodPositionForBlueGoal(Pose robotPose,
                                                    double shooterVelocityTicksPerSecond) {
        return HOOD_ANGLE_LUT.getHoodPosition(
                distanceToBlueGoal(robotPose),
                shooterVelocityTicksPerSecond
        );
    }
}
