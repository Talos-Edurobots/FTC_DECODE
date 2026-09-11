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

@Autonomous(name = "Auto Ioanna", group = "Autonomous")
public class AutoIoanna extends LinearOpMode {

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Follower follower;
    private final Pose centerPose = new Pose(71, 71, Math.toRadians(180)); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(20, 71, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose gatePose = new Pose (19, 71, Math.toRadians(180)); // Final Pose of our robot, off the starting line
    private final Pose starterPose = new Pose (71, 9, Math.toRadians(90));


    //defining our PathChains
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, openGate, toCenter, leave;



    public void buildPaths() {

         toCenter = follower.pathBuilder()
                .addPath(new BezierLine(starterPose, centerPose))
                .setLinearHeadingInterpolation(starterPose.getHeading(), centerPose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(centerPose, gatePose))
                .setLinearHeadingInterpolation(centerPose.getHeading(), gatePose.getHeading())
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(centerPose, scorePose))
                .setLinearHeadingInterpolation(centerPose.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierLine(scorePose, gatePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                  if(!follower.isBusy()) {
                    follower.followPath(toCenter);
                    setPathState(1);
                  }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(openGate);
                    setPathState(-1);
                }
                break;
            case -1:
                break;
        }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = PPConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(starterPose);

        waitForStart();
        //on start
        opmodeTimer.resetTimer();
        follower.setMaxPower(0.5);
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("followerBusy", follower.isBusy());
            telemetry.update();
        }
    }

}
