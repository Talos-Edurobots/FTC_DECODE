package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;

/**
 * Shared shooter calibration LUTs and goal-distance helpers.
 */
public final class ShooterHoodLuts {
    public static Pose BLUE_GOAL_POSE = new Pose(15.0, 128.0);

    public static final ShooterVelocityLut SHOOTER_VELOCITY_LUT = new ShooterVelocityLut(
        ShooterVelocityLut.sample(45, 1250),
        ShooterVelocityLut.sample(115, 1500)
    );

    public static final HoodAngleLut HOOD_ANGLE_LUT = new HoodAngleLut(
            HoodAngleLut.sample(45, 1250.0, 0.10),
            HoodAngleLut.sample(115, 1500, 0.13)
    );

    private ShooterHoodLuts() {}

    public static double distanceToGoal(Pose robotPose, boolean isRed) {
        return ShooterVelocityLut.distanceToGoal(robotPose, BLUE_GOAL_POSE);
    }

    public static double distanceToGoal(Pose robotPose, Pose goalPose) {
        return ShooterVelocityLut.distanceToGoal(robotPose, goalPose);
    }

    public static double getShooterVelocityForGoal(Pose robotPose, boolean isRed) {
        return SHOOTER_VELOCITY_LUT.getTargetVelocity(distanceToGoal(robotPose, isRed));
    }

    public static double getHoodPositionForBlueGoal(Pose robotPose,
                                                    double shooterVelocityTicksPerSecond, boolean isRed) {
        return HOOD_ANGLE_LUT.getHoodPosition(
                distanceToGoal(robotPose, isRed),
                shooterVelocityTicksPerSecond
        );
    }
}
