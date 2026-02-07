package org.firstinspires.ftc.teamcode.pedroPathing.main.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.RobotConstants;

public class PPConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-30.1)
            .lateralZeroPowerAcceleration(-53) // to check
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.04, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0, 0.6, 0.01))
            .centripetalScaling(0.005)
            .mass(9.7);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConstants.RIGHT_FRONT_CONFIG.getHardwareName())
            .rightRearMotorName(RobotConstants.RIGHT_BACK_CONFIG.getHardwareName())
            .leftRearMotorName(RobotConstants.LEFT_BACK_CONFIG.getHardwareName())
            .leftFrontMotorName(RobotConstants.LEFT_FRONT_CONFIG.getHardwareName())
            .leftFrontMotorDirection(RobotConstants.LEFT_FRONT_CONFIG.getDirection())
            .leftRearMotorDirection(RobotConstants.LEFT_BACK_CONFIG.getDirection())
            .rightFrontMotorDirection(RobotConstants.RIGHT_FRONT_CONFIG.getDirection())
            .rightRearMotorDirection(RobotConstants.RIGHT_BACK_CONFIG.getDirection())
            .xVelocity(70.4635)
            .yVelocity(59.7103);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.78) // check
            .strafePodX(3.34) // check
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