package org.firstinspires.ftc.teamcode.pedroPathing.main.auto; // make sure this aligns with class location

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Follower follower;
    private HardwareManager hwManager;
    private Intake intake;
    private Flickers flickers;
    private Turret turret;
    private Shooter shooter;
    private Timer pathTimer, actionTimer, opmodeTimer, flickerTimer;

    private int pathState;
    private int flickerState;
    private boolean flickersBusy = false;
    private final Pose startPose = new Pose(48, 135, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 85, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(38, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1IntakePose = new Pose(18, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(45, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupIntake2Pose = new Pose(16, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose score2ControlPos = new Pose(50, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(40, 35, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupIntake3Pose = new Pose(18, 35, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, pickup1IntakePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1IntakePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1IntakePose, scorePose))
                .setLinearHeadingInterpolation(pickup1IntakePose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickupIntake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickupIntake2Pose.getHeading())
                .addPath(new BezierLine(pickupIntake2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickupIntake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, score2ControlPos, scorePose))
                .setLinearHeadingInterpolation(pickupIntake2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, pickupIntake3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickupIntake3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupIntake3Pose, scorePose))
                .setLinearHeadingInterpolation(pickupIntake3Pose.getHeading(), scorePose.getHeading())
                .build();
    }
    public void shootArtifacts() {
        flickersBusy = true;
        switch (flickerState) {
            case 0:
                /* You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && !shooter.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    flickers.rightFlick(true);
                    setFlickerState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .3) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    flickers.rightFlick(false);
                    setFlickerState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .2) {
                    /* Score Sample */
                    intake.setCurrentState(Intake.IntakeState.INTAKE);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setFlickerState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .5) {

                    flickers.rightFlick(true);
                    setFlickerState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .3) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    flickers.rightFlick(false);
                    setFlickerState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .3) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //                    intake.setCurrentState(Intake.IntakeState.STOP);
                    flickers.leftFlick(true);
                    setFlickerState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    flickers.leftFlick(false);
                    setFlickerState(7);
                }
                break;
            case 7:
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    flickers.rightFlick(true);
                    setFlickerState(8);
                }
                break;
            case 8:
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    flickers.rightFlick(false);
                    setFlickerState(0);
                    flickersBusy = false;
                }
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                shooter.run(true);
                shooter.setHoodAngle(.4);
                turret.setAngleRadians(Math.toRadians(-50));
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && !shooter.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(2);
                }
                break;
            case 2:
                shootArtifacts();
                if (!flickersBusy) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(grabPickup2);
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                setPathState(4);
                break;
            case 4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.1){
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    setPathState(5);
                }
                break;
            case 5:
                follower.followPath(scorePickup2);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(7);
                    }
                }
                break;
            case 7:
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                follower.followPath(grabPickup1);
                setPathState(8);
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    follower.followPath(scorePickup1);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(10);
                    }
                }
                break;
            case 10:
                flickers.leftFlick(true);
                setPathState(11);
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > .5) {
                    flickers.leftFlick(false);
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setFlickerState(int fState) {
        flickerState = fState;
        flickerTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        opmodeTimer.resetTimer();
        hwManager.update();
        follower.update();
        MotorConfig.setDt(opmodeTimer.getElapsedTimeSeconds());
        intake.update();
        shooter.update(opmodeTimer.getElapsedTimeSeconds());
        turret.loop();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetryM.addData("path state", pathState);
        telemetryM.addData("flicker state", flickerState);
        telemetryM.addData("flicker busy", flickersBusy);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("shooter busy", shooter.isBusy());
        telemetryM.addData("Shooter vel", shooter.getVelocity());
        telemetryM.addData("Shooter target", shooter.getTargetVelocity());
        telemetryM.addData("path timer", pathTimer.getElapsedTime());
        telemetryM.addData("flicker timer", flickerTimer.getElapsedTime());
        telemetryM.update(telemetry);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        flickerTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hwManager = new HardwareManager(hardwareMap);

        turret = new Turret(hardwareMap);
        turret.init();

        intake = new Intake(hardwareMap);
        intake.init();
        flickers = new Flickers();
        flickers.init(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shooter.init();
        Shooter.targetVelocity = 1200;
        follower = PPConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}