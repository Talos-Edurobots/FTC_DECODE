package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PPConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-30.1)
            .lateralZeroPowerAcceleration(-53) // to check
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.04, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0, 0.6, 0.01))
            .centripetalScaling(0.005)
            .mass(7.3);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConstants.RIGHT_FRONT_NAME)
            .rightRearMotorName(RobotConstants.RIGHT_BACK_NAME)
            .leftRearMotorName(RobotConstants.LEFT_BACK_NAME)
            .leftFrontMotorName(RobotConstants.LEFT_FRONT_NAME)
            .leftFrontMotorDirection(RobotConstants.LEFT_FRONT_DIRECTION)
            .leftRearMotorDirection(RobotConstants.LEFT_BACK_DIRECTION)
            .rightFrontMotorDirection(RobotConstants.RIGHT_FRONT_DIRECTION)
            .rightRearMotorDirection(RobotConstants.RIGHT_BACK_DIRECTION)
            .xVelocity(70.4635)
            .yVelocity(59.7103);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.4) // check
            .strafePodX(4) // check
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(RobotConstants.PINPOINT_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(RobotConstants.PINPOINT_FORWARD_DIRECTION)
            .strafeEncoderDirection(RobotConstants.PINPOINT_STRAFE_DIRECTION);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}