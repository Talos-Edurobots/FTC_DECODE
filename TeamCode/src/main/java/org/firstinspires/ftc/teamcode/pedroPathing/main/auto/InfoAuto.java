package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotPoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old.SoloShortAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ShooterHoodLuts;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

public class InfoAuto {
    boolean transferBusy = false, isBlue, cycle = false;
    int transferState, pathState, cycleState;
    //    Hang hang;
    static Follower follower;
    HardwareMap hwMap;
    Telemetry telemetry;
    double distance;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private HardwareManager hwManager;
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;
    private Timer pathTimer, actionTimer, opmodeTimer, transferTimer, cycleTimer;
    private SoloShortAuto auto;
    private  Pose startPose = new Pose(48, 9, Math.toRadians(180)); // Start Pose of our robot.
    private  Pose scorePose = new Pose(50, 18, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //    private  Pose gatePose = new Pose(17, 75, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
//    private  Pose gateControlPose1 = new Pose(25, 80, Math.toRadians(180)); // Control point for the Bezier curve to open the gate.
    private  Pose pickup1Pose = new Pose(38, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup1IntakePose = new Pose(18.5, 86, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup2Pose = new Pose(45, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake2Pose = new Pose(18, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose score2ControlPos = new Pose(57, 72, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickup3Pose = new Pose(40, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake3Pose = new Pose(11, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //    private  Pose score2ndPose = new Pose(60, 74, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose pickupHuman = new Pose(9, 11, Math.toRadians(-90)); // Position to pick up the human player drop.
    private Pose pickupHumanPrePose = new Pose(9,  25, Math.toRadians(-90)); // Position to pick up the human player drop.
    private Pose pickupHumanPreControlPose = new Pose(30,  30); // Position to pick up the human player drop.
    private Pose grabAgainPose = new Pose(12, 11, Math.toRadians(180));
    private Pose grabAgainControlPose = new Pose(26, 10);
    private Pose pickUpHumanBack = new Pose(30, 10, Math.toRadians(180));
    private  Pose parkingPose = new Pose(30, 30, Math.toRadians(180)); // Parking Pose of our robot. It is in the warehouse facing forward.
    private Pose backPose = new Pose(20, 12, Math.toRadians(180));
    private Pose scoreFromGrabAgainCOntrolPose = new Pose(33, 20);
    private Path scorePreload, openGate, park;

    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabHuman, scoreHuman, stepBack, grabAgain, scoreFromGrabAgain, grabStepBackToPickupHuman, scoreToStepBack, scoreFromPickupHuman, grabHumanAgain;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        scoreFromPickupHuman = follower.pathBuilder()
                .addPath(new BezierLine(pickupHuman, scorePose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();



        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, pickupIntake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickupIntake2Pose.getHeading())
                .setGlobalDeceleration()
                .build();

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
                .addPath(new BezierCurve(scorePose, pickupHumanPreControlPose, pickupHumanPrePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupHumanPrePose.getHeading())
                .addPath(new BezierLine(pickupHumanPrePose, pickupHuman))
                .setLinearHeadingInterpolation(pickupHumanPrePose.getHeading(), pickupHuman.getHeading())
                .build();

        grabHumanAgain = follower.pathBuilder()
                .addPath(new BezierLine(pickupHuman, pickupHumanPrePose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), pickupHumanPrePose.getHeading())
                .addPath(new BezierLine(pickupHumanPrePose, pickupHuman))
                .setLinearHeadingInterpolation(pickupHumanPrePose.getHeading(), pickupHuman.getHeading())
                .setGlobalDeceleration()
                .build();

        stepBack = follower.pathBuilder()
                .addPath(new BezierLine(pickupHuman, backPose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), backPose.getHeading())
                .build();

        grabAgain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        backPose,
                        grabAgainControlPose,
                        grabAgainPose
                    )
                )
                .setLinearHeadingInterpolation(backPose.getHeading(), grabAgainPose.getHeading())
//                .setReversed()
                .build();
        scoreToStepBack = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, backPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), backPose.getHeading())
                .build();
        grabStepBackToPickupHuman = follower.pathBuilder()
                .addPath(new BezierLine(backPose, pickupHuman))
                .setLinearHeadingInterpolation(backPose.getHeading(), pickupHuman.getHeading())
                .build();
        scoreFromGrabAgain = follower.pathBuilder()
                .addPath(new BezierCurve(grabAgainPose, scoreFromGrabAgainCOntrolPose , scorePose))
                .setLinearHeadingInterpolation(grabAgain.getFinalHeadingGoal(), scorePose.getHeading())
                .build();

        scoreHuman = follower.pathBuilder()
                .addPath(new BezierLine(pickupHuman, scorePose))
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        park = new Path(new BezierLine(scorePose, parkingPose));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkingPose.getHeading());
    }

    private void shootArtifacts() {
        transferBusy = true;
        switch (transferState) {
            case 0:
                transfer.setState(Transfer.TransferState.SHOOT);
                setTransferState(1);
                break;
            case 1:
                if (transferTimer.getElapsedTimeSeconds() > .8) {
                    shooter.setHoodAngle(ShooterHoodLuts.HOOD_ANGLE_LUT.getHoodPosition(distance, shooter.getVelocity()));
//                    transfer.setState(Transfer.TransferState.STOP);
                    transferBusy = false;
                    setTransferState(0);
                }
        }
    }
    private void setAlliance(boolean isBlue) {
        if (isBlue) return;
        scoreFromGrabAgainCOntrolPose = scoreFromGrabAgainCOntrolPose.mirror();
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
        pickupHumanPrePose = pickupHumanPrePose.mirror();
        pickupHumanPreControlPose = pickupHumanPreControlPose.mirror();
        grabAgainPose = grabAgainPose.mirror();
        grabAgainControlPose = grabAgainControlPose.mirror();
        pickUpHumanBack = pickUpHumanBack.mirror();
        backPose = backPose.mirror();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
//                shooter.run(true);
//                shooter.setHoodAngle(.21);
//                turret.setAngleRadians(Math.toRadians(isBlue ? -49: 49));
                prepareForShot(scorePose);
                setPathState(1);
                break;
            case 1:

        /* You could check for
        - Follower State: "if(!follower.isBusy()) {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
        */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(((!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>2) && !shooter.isBusy() && !turret.isBusy()) && pathTimer.getElapsedTimeSeconds()>20) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(15);
                    }
                }
                break;
            case 3:
                transfer.setState(Transfer.TransferState.COLLECT);
                follower.followPath(grabPickup3);
                shooter.setIdle(true);
                setPathState(4);
            case 4:
                if (!follower.isBusy() || transfer.isFull() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scorePickup3);
                    prepareForShot(scorePose);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()||pathTimer.getElapsedTimeSeconds()>3) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(6);
                    }
                }
                break;
            case 6:
                setPathState(7);
            case 7:
                humanCycleShots();
                if(!cycle){
                    setPathState(8);
                }
                break;
            case 8:
//                follower.followPath(scoreHuman);
                shortCycleShots();
                if (!cycle) setPathState(-1);
                break;
            case 9:
                shortCycleShots();
                if (!cycle) setPathState(-1);
            case 15:
                follower.followPath(park);
                setPathState(-1);
                break;
            case -1:
                shooter.run(false);
                transfer.stop();
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setTransferState(int fState) {
        transferState = fState;
        transferTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/

    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        opmodeTimer.resetTimer();
        hwManager.update();
        follower.update();
        MotorConfig.setDt(opmodeTimer.getElapsedTimeSeconds());

        autonomousPathUpdate();
        transfer.update();
        shooter.update();
        turret.loop();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        // Feedback to Driver Hub for debugging
        telemetryM.addData("is alliance blue", isBlue);
        telemetryM.addData("path state", pathState);
        telemetryM.addData("cycle state", cycleState);
        telemetry.addData("shooter current", shooter.getCurrent1()+shooter.getCurrent2());
        telemetryM.addData("intake current", transfer.getCurrent());
        telemetryM.addData("follower busy", follower.isBusy());
        telemetryM.addData("flicker state", transferState);
        telemetryM.addData("flicker busy", transferBusy);
        telemetryM.addData("transfer state", transfer.getState());
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("velocity", follower.getVelocity().toString());
        telemetryM.addData("shooter busy", shooter.isBusy());
        telemetryM.addData("Shooter vel", shooter.getVelocity());
        telemetryM.addData("Shooter target", shooter.getTargetVelocity());
        telemetryM.addData("path timer", pathTimer.getElapsedTime());
        telemetryM.addData("flicker timer", transferTimer.getElapsedTime());
        telemetryM.update(telemetry);

        RobotPoseStorage.setPose(follower.getPose());
    }

    /** This method is called once at the init of the OpMode. **/
    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isBlue) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isBlue = isBlue;

        pathTimer = new Timer();
        transferTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        cycleTimer = new Timer();
        cycleTimer.resetTimer();


        hwManager = new HardwareManager(hwMap);

        turret = new Turret(hwMap);
        turret.init();

        transfer = new Transfer(hwMap);
        transfer.init(hwMap);

        shooter = new Shooter(hwMap);
        shooter.init();
//        Shooter.targetVelocity = 1250;

        setAlliance(isBlue);
        follower = PPConstants.createFollower(hwMap);
        buildPaths();

        follower.setStartingPose(startPose);
    }

    public void stop() {
        RobotPoseStorage.setPose(follower.getPose());
    }

    public void prepareForShot(Pose scorePose) {
        shooter.setIdle(false);
        distance = ShooterHoodLuts.distanceToGoal(scorePose, !isBlue);
        Shooter.targetVelocity = ShooterHoodLuts.SHOOTER_VELOCITY_LUT.getTargetVelocity(distance);
        shooter.setHoodAngle(ShooterHoodLuts.HOOD_ANGLE_LUT.getHoodPosition(distance, Shooter.targetVelocity));
        turret.lookToGoal(scorePose, !isBlue);
    }
    public void humanCycleShots() {
        cycle = true;
        switch (cycleState) {
            case 0:
                transfer.setState(Transfer.TransferState.COLLECT);
                follower.followPath(grabHuman);
                shooter.setIdle(true);
                setCycleState(1);
                break;
            case 1:
                if (cycleTimer.getElapsedTimeSeconds() > 3 || !follower.isBusy()) {
                    follower.breakFollowing();
                    transfer.setState(Transfer.TransferState.COLLECT);
                    setCycleState(2);
                }
                transfer.setState(Transfer.TransferState.COLLECT);
                break;
            case 2:
                if (cycleTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(grabHuman);
                    setCycleState(3);
                }
                transfer.setState(Transfer.TransferState.COLLECT);
                break;
            case 3:
                if (!follower.isBusy() || cycleTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabHumanAgain);
                    transfer.setState(Transfer.TransferState.COLLECT);
                    setCycleState(4);
                }
                transfer.setState(Transfer.TransferState.COLLECT);
                break;
            case 4:
                if (!follower.isBusy() || transfer.isFull() || cycleTimer.getElapsedTimeSeconds() > 3) {
//                    follower.followPath(grabAgain);
                    transfer.setState(Transfer.TransferState.COLLECT);
                    setCycleState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scoreFromPickupHuman);
                    prepareForShot(scorePose);
                    transfer.setState(Transfer.TransferState.STOP);
                    setCycleState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && cycleTimer.getElapsedTimeSeconds() > 3) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setCycleState(0);
                        cycle = false;
                    }
                }
                break;
        }
    }
    public void shortCycleShots() {
        cycle = true;
        switch (cycleState) {
            case 0:
                transfer.setState(Transfer.TransferState.COLLECT);
                follower.followPath(scoreToStepBack);
                shooter.setIdle(true);
                setCycleState(1);
                break;
            case 1:
                if (!follower.isBusy() || cycleTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(stepBack);
                    setCycleState(4);
                }
                transfer.setState(Transfer.TransferState.COLLECT);
                break;
            case 4:
                if (!follower.isBusy() || transfer.isFull()) {
                    follower.followPath(grabAgain);
                    transfer.setState(Transfer.TransferState.COLLECT);
                    setCycleState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(scoreFromGrabAgain);
                    prepareForShot(scorePose);
                    transfer.setState(Transfer.TransferState.STOP);
                    setCycleState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && cycleTimer.getElapsedTimeSeconds() > 3) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setCycleState(0);
                        cycle = false;
                    }
                }
                break;
        }
    }
    public void setCycleState(int state) {
        cycleState = state;
        cycleTimer.resetTimer();
    }

}

