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
import org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old.SoloShortAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotPoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

import java.util.HashMap;

public class AutoV2 {
    boolean transferBusy = false, isBlue;
    int transferState, pathState;
    //    Hang hang;
    static Follower follower;
    HardwareMap hwMap;
    Telemetry telemetry;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private HardwareManager hwManager;
    private Intake intake;
    private Gate gate;
    private Turret turret;
    private Shooter shooter;
    private Timer pathTimer, actionTimer, opmodeTimer, transferTimer;
    private SoloShortAuto auto;
    private  Pose startPose = new Pose(25, 129, Math.toRadians(233)); // Start Pose of our robot.
    private  Pose scorePreloadPose = new Pose(48, 100, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose gateWithoutGrabPose = new Pose(20, 62, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
    private Pose gateWithoutGrabPoseControl1 = new Pose(48, 60); // Position of the gate that we need to open to access the artifacts.
    private Pose gateWithoutGrabPoseControl2 = new Pose(42, 60); // Position of the gate that we need to open to access the artifacts.
    private  Pose scorePose = new Pose(60, 78, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose gateGrabPose = new Pose(14, 56, Math.toRadians(155)); // Position of the gate that we need to open to access the artifacts.
    private Pose gateIntermediatePose = new Pose(40, 60, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
    private Pose gateIntermediateControlPose = new Pose(54, 67, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
    private  Pose pickup1Pose = new Pose(38, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup1IntakePose = new Pose(18.5, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup2Pose = new Pose(45, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake2Pose = new Pose(15, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose score2ControlPos = new Pose(57, 72, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickup3Pose = new Pose(40, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake3Pose = new Pose(12, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private  Pose score2ndPose = new Pose(60, 74, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private  Pose parkingPose = new Pose(50, 70, Math.toRadians(180)); // Parking Pose of our robot. It is in the warehouse facing forward.

    private Path scorePreload, openGate, park;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabHuman, scoreHuman, grabGateFromScore, scoreFromGate;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePreloadPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPose, gateWithoutGrabPoseControl1, gateWithoutGrabPoseControl2, gateWithoutGrabPose))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), gateWithoutGrabPose.getHeading())
                .setGlobalDeceleration()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gateWithoutGrabPose, scorePose))
                .setLinearHeadingInterpolation(gateWithoutGrabPose.getHeading(), scorePose.getHeading())
                .build();

        grabGateFromScore = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gateIntermediateControlPose, gateIntermediatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateIntermediatePose.getHeading())
                .addPath(new BezierLine(gateIntermediatePose, gateGrabPose))
                .setLinearHeadingInterpolation(gateIntermediatePose.getHeading(), gateGrabPose.getHeading())
                .build();
        scoreFromGate = follower.pathBuilder()
                .addPath(new BezierLine(gateGrabPose, scorePose))
                .setLinearHeadingInterpolation(gateGrabPose.getHeading(), scorePose.getHeading())
                .build();
    }
    private void shootArtifacts() {
        transferBusy = true;
        switch (transferState) {
            case 0:
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                gate.deactivate();
                setTransferState(1);
                break;
            case 1:
                if (transferTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    gate.activate();
                    transferBusy = false;
                    setTransferState(0);
                }
        }
    }
    private void setAlliance(boolean isBlue) {
        if (isBlue) return;
        startPose = startPose.mirror();
        scorePreloadPose = scorePreloadPose.mirror();
        gateWithoutGrabPose = gateWithoutGrabPose.mirror();
        gateWithoutGrabPoseControl1 = gateWithoutGrabPoseControl1.mirror();
        gateWithoutGrabPoseControl2 = gateWithoutGrabPoseControl2.mirror();
        scorePose = scorePose.mirror();
        gateGrabPose = gateGrabPose.mirror();
        gateIntermediatePose = gateIntermediatePose.mirror();
        gateIntermediateControlPose = gateIntermediateControlPose.mirror();
        pickup1Pose = pickup1Pose.mirror();
        pickup1IntakePose = pickup1IntakePose.mirror();
        pickup2Pose = pickup2Pose.mirror();
        pickupIntake2Pose = pickupIntake2Pose.mirror();
        score2ControlPos = score2ControlPos.mirror();
        pickup3Pose = pickup3Pose.mirror();
        pickupIntake3Pose = pickupIntake3Pose.mirror();
        score2ndPose = score2ndPose.mirror();
        parkingPose = parkingPose.mirror();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                shooter.run(true);
                shooter.setHoodAngle(.4);
                turret.setAngleRadians(Math.toRadians(isBlue ? -49: 49));
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
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                follower.followPath(grabPickup2);
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                setPathState(4);
                break;
            case 4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.4){
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    setPathState(5);
                }
                break;
            case 5:
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                follower.followPath(grabGateFromScore);
                Shooter.targetVelocity = 1350;
                shooter.setHoodAngle(.3);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()/* && pathTimer.getElapsedTimeSeconds() > .05*/) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    turret.setAngleRadians(Math.toRadians(isBlue?-49:49));
                    follower.followPath(scoreFromGate);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(-1);
                    }
                }
                break;
            case 11:
                follower.followPath(grabPickup3);
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                setPathState(12);
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {

                    follower.followPath(scorePickup3);
//                        turret.setAngleRadians(Math.toRadians(isBlue ? -38:38));
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    shootArtifacts();
                    if (!transferBusy) {
                        setPathState(15);
                    }
                }
                break;
            case 15:
                follower.followPath(park);
                setPathState(-1);
                break;
            case -1:
                shooter.run(false);
                intake.setCurrentState(Intake.IntakeState.STOP);
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
        intake.update();
        shooter.update();
        turret.loop();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetryM.addData("is alliance blue", isBlue);
        telemetryM.addData("path state", pathState);
        telemetryM.addData("flicker state", transferState);
        telemetryM.addData("flicker busy", transferBusy);
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

        hwManager = new HardwareManager(hwMap);

        turret = new Turret(hwMap);
        turret.init();

        intake = new Intake(hwMap);
        intake.init();

        gate = new Gate();
        gate.init(hwMap);

        shooter = new Shooter(hwMap);
        shooter.init();
        Shooter.targetVelocity = 1250;

        setAlliance(isBlue);
        follower = PPConstants.createFollower(hwMap);
        buildPaths();

        follower.setStartingPose(startPose);
    }

    public void stop(HashMap blackboard) {
        RobotPoseStorage.setPose(follower.getPose());
        blackboard.put(RobotConstants.ALLIANCE_KEY, isBlue);
        blackboard.put(RobotConstants.FOLLOWER_KEY, follower);
    }

}

