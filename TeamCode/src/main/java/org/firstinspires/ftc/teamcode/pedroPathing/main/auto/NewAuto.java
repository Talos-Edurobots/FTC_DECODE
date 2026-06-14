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
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

import java.util.HashMap;

public class NewAuto {
    boolean flickersBusy = false, isBlue;
    int flickerState, pathState;
//    Hang hang;
    static Follower follower;
    HardwareMap hwMap;
    Telemetry telemetry;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private HardwareManager hwManager;
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;
    private Timer pathTimer, actionTimer, opmodeTimer, flickerTimer;
    private SoloShortAuto auto;
    private  Pose startPose = new Pose(23, 136, Math.toRadians(233)); // Start Pose of our robot.
    private  Pose scorePose = new Pose(48, 85, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private  Pose gatePose = new Pose(17.4, 80, Math.toRadians(180)); // Position of the gate that we need to open to access the artifacts.
    private  Pose gateControlPose1 = new Pose(25, 80, Math.toRadians(180)); // Control point for the Bezier curve to open the gate.
    private  Pose pickup1Pose = new Pose(38, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup1IntakePose = new Pose(18.5, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private  Pose pickup2Pose = new Pose(45, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickupIntake2Pose = new Pose(10, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private Pose pickup2ControlPose = new Pose(52, 58);
    private  Pose score2ControlPos = new Pose(57, 72, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private  Pose pickup3Pose = new Pose(40, 40, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Pose pickup3ControlPose = new Pose(52, 45);
    private  Pose pickupIntake3Pose = new Pose(11, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private  Pose score2ndPose = new Pose(60, 74, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private  Pose parkingPose = new Pose(50, 70, Math.toRadians(180)); // Parking Pose of our robot. It is in the warehouse facing forward.

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
                .addPath(new BezierCurve(scorePose, pickup2ControlPose, pickupIntake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickupIntake2Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        openGate = new Path(new BezierCurve(pickup1IntakePose, gateControlPose1, gatePose));
        openGate.setLinearHeadingInterpolation(pickup1IntakePose.getHeading(), gatePose.getHeading());
        openGate.setVelocityConstraint(10);

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupIntake2Pose, score2ndPose))
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
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(pickup1IntakePose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(score2ndPose, pickup3ControlPose, pickupIntake3Pose))
                .setLinearHeadingInterpolation(score2ndPose.getHeading(), pickupIntake3Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupIntake3Pose, score2ndPose))
                .setLinearHeadingInterpolation(pickupIntake3Pose.getHeading(), score2ndPose.getHeading())
                .setGlobalDeceleration()
                .build();

        park = new Path(new BezierLine(score2ndPose, parkingPose));
        park.setLinearHeadingInterpolation(score2ndPose.getHeading(), parkingPose.getHeading());
    }
    //    public SoloShortAuto get() {
//        if (auto == null) {
//            auto = new SoloShortAuto();
//        }
//        return auto;
//    }
    private void shootArtifacts() {
        flickersBusy = true;
        switch (flickerState) {
            case 0:
                transfer.shoot();
                setFlickerState(1);
                break;
            case 1:
                if (flickerTimer.getElapsedTimeSeconds() > 1.5) {
                    transfer.stop();
                    flickersBusy = false;
                    setFlickerState(0);
            }
        }
    }
    private void setAlliance(boolean isBlue) {
        if (isBlue) return;
        startPose = startPose.mirror();
        scorePose = scorePose.mirror();
        gatePose = gatePose.mirror();
        gateControlPose1 = gateControlPose1.mirror();
        pickup1Pose = pickup1Pose.mirror();
        pickup1IntakePose = pickup1IntakePose.mirror();
        pickup2Pose = pickup2Pose.mirror();
        pickupIntake2Pose = pickupIntake2Pose.mirror();
        pickup2ControlPose = pickup2ControlPose.mirror();
        score2ControlPos = score2ControlPos.mirror();
        pickup3Pose = pickup3Pose.mirror();
        pickupIntake3Pose = pickupIntake3Pose.mirror();
        pickup3ControlPose = pickup3ControlPose.mirror();
        score2ndPose = score2ndPose.mirror();
        parkingPose = parkingPose.mirror();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                shooter.run(true);
                shooter.setHoodAngle(.1);
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
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                follower.followPath(grabPickup1);
                transfer.collect();
                setPathState(4);
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>2){
                    transfer.stop();
                    setPathState(5);
                }
                break;
            case 5:
                follower.followPath(openGate);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                transfer.collect();
                follower.followPath(grabPickup2);
                Shooter.targetVelocity = 1350;
                shooter.setHoodAngle(0.1);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>4) {
                    transfer.stop();
                    turret.setAngleRadians(Math.toRadians(isBlue?-49:49));
                    follower.followPath(scorePickup2);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    shootArtifacts();
                    if (!flickersBusy) {
                        setPathState(11);
                    }
                }
                break;
            case 11:
                follower.followPath(grabPickup3);
                transfer.collect();
                setPathState(12);
                break;
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4 || transfer.isFull()) {

                    follower.followPath(scorePickup3);
//                        turret.setAngleRadians(Math.toRadians(isBlue ? -38:38));
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    transfer.stop();
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()/* || pathTimer.getElapsedTimeSeconds() > 0.4*/) {
                    shootArtifacts();
                    if (!flickersBusy) {
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
                transfer.stop();
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

        autonomousPathUpdate();
        transfer.update();
        shooter.update();
        turret.loop();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        // Feedback to Driver Hub for debugging
        telemetryM.addData("is alliance blue", isBlue);
        telemetryM.addData("path state", pathState);
        telemetryM.addData("flicker state", flickerState);
        telemetryM.addData("flicker busy", flickersBusy);
        telemetryM.addData("transfer state", transfer.getState());
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

        RobotPoseStorage.setPose(follower.getPose());
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

        turret = new Turret(hwMap);
        turret.init();

        transfer = new Transfer(hwMap);
        transfer.init(hwMap);

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

