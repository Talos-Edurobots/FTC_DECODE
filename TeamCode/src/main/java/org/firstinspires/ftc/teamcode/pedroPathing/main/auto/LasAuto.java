package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;

@Autonomous(name = "LasAuto")
public class LasAuto extends LinearOpMode {
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Follower follower;
    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));
    private final Pose getStartPose1 = new Pose(72, 72, Math.toRadians(180));// Start Pose of our robot. This is against the goal facing AWAY
    private final Pose gatePose = new Pose(16   , 72, Math.toRadians(180)); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(72, 72, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose (60, 105); // Final Pose of our robot, off the starting line
    //defining our PathChains
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, opengate,  pathline1, grabPickup3, scorePickup3, leave;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        pathline1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setTangentHeadingInterpolation()
                .build();
        opengate = follower.pathBuilder()
                .addPath(new BezierLine(getStartPose1, gatePose))
                .setTangentHeadingInterpolation()
                .build();


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 54), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(60, 54), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose,new Pose(60, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our leave PathChain. We are using a single path using a BezierLine (straight line).
         * We use Constant Interpolation here instead of Linear*/
        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0 :
                follower.followPath(pathline1);
                setPathState(1);
                break;
            case 1 :
                follower.followPath(opengate);
                setPathState(1);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = PPConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        //on start
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setMaxPower(0.4);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

}
