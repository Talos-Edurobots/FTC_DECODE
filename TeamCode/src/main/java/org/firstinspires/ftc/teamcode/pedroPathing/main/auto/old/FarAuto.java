package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

import java.util.HashMap;

public class FarAuto {
    boolean flickersBusy = false, isBlue;
    int flickerState, pathState;
    Hang hang;
    static Follower follower;
    HardwareMap hwMap;
    Telemetry telemetry;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private HardwareManager hwManager;
    private Intake intake;
    private Flickers flickers;
    private Turret turret;
    private Shooter shooter;
    private Timer pathTimer, actionTimer, opmodeTimer, flickerTimer;
    private SoloShortAuto auto;
    private  Pose startPose = new Pose(56, 8, Math.toRadians(180)); // Start Pose of our robot.
    private  Pose scorePose = new Pose(56, 18, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private  Pose gatePose = new Pose(17, 75, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
//    private  Pose gateControlPose1 = new Pose(25, 80, Math.toRadians(180)); // Control point for the Bezier curve to open the gate.
    private  Pose pickup1Pose = new Pose(38, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup1IntakePose = new Pose(18.5, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup2Pose = new Pose(45, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake2Pose = new Pose(18, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose score2ControlPos = new Pose(57, 72, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickup3Pose = new Pose(40, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake3Pose = new Pose(12, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private  Pose score2ndPose = new Pose(60, 74, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose pickupHuman = new Pose(11, 9, Math.toRadians(180)); // Position to pick up the human player drop.
    private  Pose parkingPose = new Pose(60, 30, Math.toRadians(180)); // Parking Pose of our robot. It is in the warehouse facing forward.
    private Pose backPose = new Pose(20, 8, Math.toRadians(180));
    private Path scorePreload, openGate, park;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabHuman, scoreHuman;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, pickupIntake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickupIntake2Pose.getHeading())
                .setGlobalDeceleration()
                .build();

//        openGate = new Path(new BezierCurve(pickup1IntakePose, gateControlPose1, gatePose));
//        openGate.setLinearHeadingInterpolation(pickup1IntakePose.getHeading(), gatePose.getHeading());
//        openGate.setVelocityConstraint(10);

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupIntake2Pose, scorePose))
                .setLinearHeadingInterpolation(pickupIntake2Pose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1IntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1IntakePose.getHeading())
                .setGlobalDeceleration()
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1IntakePose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, pickupIntake3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickupIntake3Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupIntake3Pose, scorePose))
                .setLinearHeadingInterpolation(pickupIntake3Pose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        grabHuman = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupHuman))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupHuman.getHeading())
                .addPath(new BezierLine(pickupHuman, backPose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), backPose.getHeading())
                .setVelocityConstraint(20)
                .addPath(new BezierLine(backPose, pickupHuman))
                .setLinearHeadingInterpolation(backPose.getHeading(), pickupHuman.getHeading())
                .setVelocityConstraint(20)
                .setGlobalDeceleration()
                .addPath(new BezierLine(pickupHuman, backPose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), backPose.getHeading())
                .setVelocityConstraint(20)
                .addPath(new BezierLine(backPose, pickupHuman))
                .setLinearHeadingInterpolation(backPose.getHeading(), pickupHuman.getHeading())
                .setVelocityConstraint(20)
                .setGlobalDeceleration()
                .build();

        scoreHuman = follower.pathBuilder()
                .addPath(new BezierLine(pickupHuman, scorePose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        park = new Path(new BezierLine(scorePose, parkingPose));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkingPose.getHeading());
    }

    //    public SoloShortAuto get() {
//        if (auto == null) {
//            auto = new SoloShortAuto();
//        }
//        return auto;
//    }
    private void setAlliance(boolean isBlue) {
        if (isBlue) return;
        startPose = startPose.mirror();
        scorePose = scorePose.mirror();
        pickup1Pose = pickup1Pose.mirror();
        pickup1IntakePose = pickup1IntakePose.mirror();
        pickup2Pose = pickup2Pose.mirror();
        pickupIntake2Pose = pickupIntake2Pose.mirror();
        score2ControlPos = score2ControlPos.mirror();
        pickup3Pose = pickup3Pose.mirror();
        pickupIntake3Pose = pickupIntake3Pose.mirror();
        parkingPose = parkingPose.mirror();
        pickupHuman = pickupHuman.mirror();
        backPose = backPose.mirror();
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
                if (flickerTimer.getElapsedTimeSeconds() > .6) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    flickers.rightFlick(false);
                    setFlickerState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .4) {
                    /* Score Sample */
                    intake.setCurrentState(Intake.IntakeState.INTAKE);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setFlickerState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .5) {

                    flickers.leftFlick(true);
                    setFlickerState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .3) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    flickers.leftFlick(false);
                    setFlickerState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .3) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //                    intake.setCurrentState(Intake.IntakeState.STOP);
                    flickers.rightFlick(true);
                    setFlickerState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    flickers.rightFlick(false);
                    setFlickerState(7);
                }
                break;
            case 7:
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    flickers.leftFlick(true);
                    setFlickerState(8);
                }
                break;
            case 8:
                if (flickerTimer.getElapsedTimeSeconds() > .5) {
                    flickers.leftFlick(false);
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
                shooter.setHoodAngle(0);
                turret.setAngleRadians(Math.toRadians(isBlue ? -67: 67));
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
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(3);
                }
            case 3:
                shootArtifacts();
                if (!flickersBusy) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(grabPickup3);
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                setPathState(5);
                break;
            case 5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.4){
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    setPathState(6);
                }
                break;
            case 6:
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    turret.setAngleRadians(Math.toRadians(isBlue ? -66: 66));
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(10);
                    }
                }
                break;
            case 10:
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                follower.followPath(grabHuman);
//                Shooter.targetVelocity = 1250;
//                shooter.setHoodAngle(.2);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()/* && pathTimer.getElapsedTimeSeconds() > .05*/) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    follower.followPath(scoreHuman);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(14);
                    }
                }
                break;
            case 14:
//                follower.followPath(grabPickup3);
//                intake.setCurrentState(Intake.IntakeState.INTAKE);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {

//                    follower.followPath(scorePickup3);
//                        turret.setAngleRadians(Math.toRadians(isBlue ? -38:38));
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    setPathState(18);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(18);
                    }
                }
                break;
            case 18:
                follower.followPath(park);
                setPathState(-1);
                break;
            case -1:
                shooter.run(false);
                intake.setCurrentState(Intake.IntakeState.STOP);
                turret.setAngleRadians(0);
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

    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        opmodeTimer.resetTimer();
        hwManager.update();
        follower.update();
        MotorConfig.setDt(opmodeTimer.getElapsedTimeSeconds());
        intake.update();
        shooter.update();
        turret.loop();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetryM.addData("is alliance blue", isBlue);
        telemetryM.addData("path state", pathState);
        telemetryM.addData("flicker state", flickerState);
        telemetryM.addData("flicker busy", flickersBusy);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("velocity", follower.getVelocity().toString());
        telemetryM.addData("shooter busy", shooter.isBusy());
        telemetryM.addData("Shooter vel", shooter.getVelocity());
        telemetryM.addData("Shooter target", shooter.getTargetVelocity());
        telemetryM.addData("path timer", pathTimer.getElapsedTime());
        telemetryM.addData("flicker timer", flickerTimer.getElapsedTime());
        telemetryM.update(telemetry);
    }

    /** This method is called once at the init of the OpMode. **/
    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isBlue) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isBlue = isBlue;

        pathTimer = new Timer();
        flickerTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hwManager = new HardwareManager(hwMap);

        hang = new Hang();
        hang.init(hwMap);

        turret = new Turret(hwMap);
        turret.init();

        intake = new Intake(hwMap);
        intake.init();

        flickers = new Flickers();
        flickers.init(hwMap);

        shooter = new Shooter(hwMap);
        shooter.init();
        Shooter.targetVelocity = 1500;

        setAlliance(isBlue);
        follower = PPConstants.createFollower(hwMap);
        buildPaths();

        follower.setStartingPose(startPose);
    }

    public void stop(HashMap blackboard) {
        blackboard.put(RobotConstants.ALLIANCE_KEY, isBlue);
        blackboard.put(RobotConstants.FOLLOWER_KEY, follower);
    }

}

