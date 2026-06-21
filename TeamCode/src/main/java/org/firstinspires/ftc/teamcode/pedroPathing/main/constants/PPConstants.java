package org.firstinspires.ftc.teamcode.pedroPathing.main.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PPConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-23.739580125906762)
            .lateralZeroPowerAcceleration(-63.39827798387032)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.025278453672773423, 0.0021575944378939543)) // (kP, kLinear, kQuadratic)
//            .translationalPIDFCoefficients(new PIDFCoefficients(.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.17, 0.02))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.03))
            .centripetalScaling(0) // 0.00035
            .mass(9.7);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName(RobotConstants.RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(RobotConstants.RIGHT_BACK_MOTOR_NAME)
            .leftRearMotorName(RobotConstants.LEFT_BACK_MOTOR_NAME)
            .leftFrontMotorName(RobotConstants.LEFT_FRONT_MOTOR_NAME)
            .leftFrontMotorDirection(RobotConstants.LEFT_FRONT_MOTOR_DIRECTION)
            .leftRearMotorDirection(RobotConstants.LEFT_BACK_MOTOR_DIRECTION)
            .rightFrontMotorDirection(RobotConstants.RIGHT_FRONT_MOTOR_DIRECTION)
            .rightRearMotorDirection(RobotConstants.RIGHT_BACK_MOTOR_DIRECTION)
            .xVelocity(66.92134346548967)
            .yVelocity(50.80548432117372);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5) // check
            .strafePodX(-2.67) // check
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(RobotConstants.PINPOINT_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(RobotConstants.PINPOINT_FORWARD_DIRECTION)
            .strafeEncoderDirection(RobotConstants.PINPOINT_STRAFE_DIRECTION);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
