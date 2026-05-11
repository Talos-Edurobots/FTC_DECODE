package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.pedropathing.geometry.Pose;

public final class RobotPoseStorage {
    private static Pose pose;

    private RobotPoseStorage() {
    }

    public static void setPose(Pose newPose) {
        pose = newPose == null
                ? null
                : new Pose(newPose.getX(), newPose.getY(), newPose.getHeading());
    }

    public static Pose getPose() {
        return pose == null
                ? null
                : new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static boolean hasPose() {
        return pose != null;
    }

    public static void clear() {
        pose = null;
    }
}
