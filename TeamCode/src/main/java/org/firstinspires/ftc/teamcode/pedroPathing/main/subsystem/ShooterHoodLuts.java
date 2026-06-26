package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;

/**
 * Shared shooter calibration LUTs and goal-distance helpers.
 */
public final class ShooterHoodLuts {
    public static Pose BLUE_GOAL_POSE = new Pose(15.0, 128.0);
    public static Pose RED_GOAL_POSE = new Pose(126.0, 128.0);

    public static final ShooterVelocityLut SHOOTER_VELOCITY_LUT = new ShooterVelocityLut(
        ShooterVelocityLut.sample(80.70,  1200.0),
        ShooterVelocityLut.sample(71.84,  1250.0),
        ShooterVelocityLut.sample(46.69,  1050.0),
        ShooterVelocityLut.sample(118.60, 1550.0),
        ShooterVelocityLut.sample(108.52, 1450.0),
        ShooterVelocityLut.sample(140,    1560.0)
    );

    public static final HoodAngleLut HOOD_ANGLE_LUT = new HoodAngleLut(
            HoodAngleLut.sample(46.69, 1100.0, 0.291),
            HoodAngleLut.sample(46.69, 1050.0, 0.437),
            HoodAngleLut.sample(71.84, 1260.0, 0.159),
            HoodAngleLut.sample(71.84, 1200.0, 0.138),
            HoodAngleLut.sample(79.91, 1250.0, 0.000),
            HoodAngleLut.sample(79.91, 1200.0, 0.188),
            HoodAngleLut.sample(33.53, 970.0, 0.500),
            HoodAngleLut.sample(118.60, 1400.0, 0.25),
            HoodAngleLut.sample(108.52, 1420.0, 0.25),
            HoodAngleLut.sample(129.25, 1500, 0.3)
        );

    private ShooterHoodLuts() {}

    public static double distanceToGoal(Pose robotPose, boolean isRed) {
        return ShooterVelocityLut.distanceToGoal(robotPose, isRed?RED_GOAL_POSE:BLUE_GOAL_POSE);
//        return ShooterVelocityLut.distanceToGoal(robotPose, BLUE_GOAL_POSE);
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
