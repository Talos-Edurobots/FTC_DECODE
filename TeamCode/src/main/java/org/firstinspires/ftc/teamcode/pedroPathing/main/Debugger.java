package org.firstinspires.ftc.teamcode.pedroPathing.main;


import android.util.Log;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorUse;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;


@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {
    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", ne -> {
                ne.add("shooter subsystem", ShooterPowerTest::new);
                ne.add("Run Intake",  () -> new MotorPowerTest(RobotConstants.INTAKE_CONFIG.copy()));
                ne.add("Run Shooter", () -> new MotorPowerTest(RobotConstants.SHOOTER_CONFIG.copy()));
                ne.add("Run Turret",  () -> new MotorPowerTest(RobotConstants.TURRET_CONFIG.copy()));
                ne.add("Run Hang", () -> new MotorPowerTest(RobotConstants.HANG_CONFIG.copy()));
                ne.add("Run Left Front Drive",  () -> new MotorPowerTest(RobotConstants.LEFT_FRONT_CONFIG.copy()));
                ne.add("Run Right Front Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_FRONT_CONFIG.copy()));
                ne.add("Run Left Back Drive",   () -> new MotorPowerTest(RobotConstants.LEFT_BACK_CONFIG.copy()));
                ne.add("Run Right Back Drive",  () -> new MotorPowerTest(RobotConstants.RIGHT_BACK_CONFIG.copy()));
            });
            s.folder("Velocity Control", vc -> {
                vc.add("Run Shooter Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.SHOOTER_CONFIG.copy()));
                vc.add("Run Intake Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.INTAKE_CONFIG.copy()));
            });
            s.folder("servo control", sc -> {
                sc.add("right servo", () -> new ServoControl(RobotConstants.RIGHT_SERVO_NAME));
                sc.add("left servo", () -> new ServoControl(RobotConstants.LEFT_SERVO_NAME));
                sc.add("right flicker", () -> new ServoControl(RobotConstants.RIGHT_FLICKER_NAME));
                sc.add("left flicker", () -> new ServoControl(RobotConstants.LEFT_FLICKER_NAME));
                sc.add("enable pwm all servos", EnableAllServoPwmOpMode::new);
            });
            s.folder("position control", pc -> {
                pc.add("turret position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG.copy()));
                pc.add("turret ka test", () -> new KaTestOpMode(RobotConstants.TURRET_CONFIG.copy()));
                pc.add("turret stick teleop", () -> new TurretStickTeleOp(RobotConstants.TURRET_CONFIG.copy()));
            });
            s.folder("ke characterization", kect -> {
                kect.add("shooter ke", KeCharacterizationOpMode::new);
//                kect.add("intake ke", () -> new KeCharacterizationOpMode(RobotConstants.INTAKE_CONFIG));
//                kect.add("turret ke", () -> new KeCharacterizationOpMode(RobotConstants.TURRET_CONFIG));
            });
            s.folder("high level", hlt -> {
                hlt.add("flicker analog control", FlickerAnalogControl::new);
                hlt.add("voltage sensor readout", VoltageSensorReadoutOpMode::new);
                hlt.add("color readout", ColorReadoutOpMode::new);
                hlt.add("led subsystem demo", LedsSubsystemDemo::new);
//                hlt.add("turret position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG));
                hlt.add("hang control", HangControl::new);
                hlt.add("robot mechanism demo", RobotMechanismDemo::new);
                hlt.add("turret simple pidf with turret", LimelightTurretAlign::new);
                hlt.add("throughput test", TestThroughput::new);
                hlt.add("collect data", CollectData::new);
            });
        });
    }

    @Override
    public void onSelect() {}
}

@Configurable
class RobotMechanismDemo extends OpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private static final double HOOD_STEP = 0.02;
    private static final double TURRET_STEP_DEGREES = 2;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    public static boolean intakeEnabled = false;
    public static boolean intakeReverse = false;
    public static boolean shooterEnabled = false;
    public static double shooterTargetVelocity = 2400;
    public static double turretTargetDegrees = 0;
    public static double hoodAngle = 0.5;

    @Override
    public void init() {
        intakeEnabled = false;
        intakeReverse = false;
        shooterEnabled = false;
        shooterTargetVelocity = 2400;
        turretTargetDegrees = 0;
        hoodAngle = 0.5;

        intake = new Intake(hardwareMap);
        intake.init();

        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(false);
        shooter.setHoodAngle(hoodAngle);

        turret = new Turret(hardwareMap);
        turret.init();

        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetryM.addLine("Robot mechanism demo ready");
        telemetryM.addLine("Use Panels to toggle intake/shooter and set turret + shooter targets");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intakeEnabled = !intakeEnabled;
            if (intakeEnabled && intakeReverse) intakeReverse = false;
        }
        if (gamepad1.bWasPressed()) {
            intakeEnabled = true;
            intakeReverse = !intakeReverse;
        }
        if (gamepad1.xWasPressed()) {
            shooterEnabled = !shooterEnabled;
        }
        if (gamepad1.dpadUpWasPressed()) {
            shooterTargetVelocity += 100;
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooterTargetVelocity = Math.max(0, shooterTargetVelocity - 100);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            turretTargetDegrees -= TURRET_STEP_DEGREES;
        }
        if (gamepad1.dpadRightWasPressed()) {
            turretTargetDegrees += TURRET_STEP_DEGREES;
        }
        if (gamepad1.leftBumperWasPressed()) {
            hoodAngle -= HOOD_STEP;
        }
        if (gamepad1.rightBumperWasPressed()) {
            hoodAngle += HOOD_STEP;
        }
        hoodAngle = Math.max(0, Math.min(0.5, hoodAngle));

        intake.setCurrentState(
                intakeEnabled
                        ? (intakeReverse ? Intake.IntakeState.OUTTAKE : Intake.IntakeState.INTAKE)
                        : Intake.IntakeState.STOP
        );
        intake.update();

        Shooter.setTargetVelocity(shooterTargetVelocity);
        shooter.run(shooterEnabled);
        shooter.setHoodAngle(hoodAngle);
        shooter.update();

        turret.setAngleRadians(Math.toRadians(turretTargetDegrees));
        turret.loop();

        telemetryM.addLine("Controls: A intake toggle, B reverse intake, X shooter toggle");
        telemetryM.addLine("Dpad up/down shooter velocity, dpad left/right turret, bumpers hood");
        telemetryM.addLine("-------------------------");
        telemetryM.addData("intake enabled", intakeEnabled);
        telemetryM.addData("intake reverse", intakeReverse);
        telemetryM.addData("intake state", intake.getCurrentState());
        telemetryM.addData("intake velocity", intake.getVelocity());
        telemetryM.addData("intake current", intake.getCurrent());
        telemetryM.addLine("-------------------------");
        telemetryM.addData("shooter enabled", shooterEnabled);
        telemetryM.addData("shooter target velocity", shooterTargetVelocity);
        telemetryM.addData("shooter velocity", shooter.getVelocity());
        telemetryM.addData("shooter power", shooter.getPower());
        telemetryM.addData("shooter current", shooter.getCurrent1());
        telemetryM.addData("hood angle", shooter.getHoodAngle());
        telemetryM.addLine("-------------------------");
        telemetryM.addData("turret target degrees", turretTargetDegrees);
        telemetryM.update(telemetry);
    }
}

@Configurable
class LedsSubsystemDemo extends OpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Leds leds;
    private double lastLoopTime = 0.0;

    public static double leftBaseColor = 0.333;
    public static double rightBaseColor = 0.722;
    public static double alertColor = 1.0;
    public static double targetLockColor = 0.5;
    public static double intakeStoppedColor = 0.0;
    public static double blinkInterval = 0.2;
    public static double holdDuration = 1.0;
    public static int blinkCount = 3;
    public static boolean useRgbIdle = false;

    @Override
    public void init() {
        leds = new Leds();
        leds.init(hardwareMap);

        leftBaseColor = 0.333;
        rightBaseColor = 0.722;
        alertColor = 1.0;
        targetLockColor = 0.5;
        intakeStoppedColor = 0.0;
        blinkInterval = 0.2;
        holdDuration = 1.0;
        blinkCount = 3;
        useRgbIdle = false;

        leds.setBase(Leds.Side.LEFT, leftBaseColor);
        leds.setBase(Leds.Side.RIGHT, rightBaseColor);

        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetryM.addLine("LED subsystem demo ready");
        telemetryM.addLine("A finite blink, B pulse right, X timed hold, Y clear");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0) {
            dt = 0;
        }

        leds.setBase(Leds.Side.LEFT, leftBaseColor);
        leds.setBase(Leds.Side.RIGHT, rightBaseColor);

        if (gamepad1.aWasPressed()) {
            leds.blink(Leds.Side.BOTH, alertColor, blinkCount, blinkInterval);
        }
        if (gamepad1.bWasPressed()) {
            leds.pulse(Leds.Side.RIGHT, rightBaseColor, alertColor, blinkInterval);
        }
        if (gamepad1.xWasPressed()) {
            leds.showColorFor(Leds.Side.LEFT, targetLockColor, holdDuration);
        }
        if (gamepad1.yWasPressed()) {
            leds.clearEffects();
        }
        if (gamepad1.leftBumperWasPressed()) {
            leds.alertLeft(alertColor, blinkCount, blinkInterval);
        }
        if (gamepad1.rightBumperWasPressed()) {
            leds.alertRight(alertColor, blinkCount, blinkInterval);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            leds.showColorFor(Leds.Side.BOTH, intakeStoppedColor, holdDuration);
        }
        if (gamepad1.dpadRightWasPressed()) {
            leds.pulse(Leds.Side.BOTH, leftBaseColor, rightBaseColor, blinkInterval);
        }
        if (gamepad1.dpadUpWasPressed()) {
            useRgbIdle = !useRgbIdle;
        }
        if (gamepad1.dpadDownWasPressed()) {
            leds.blink(Leds.Side.LEFT, alertColor, intakeStoppedColor, blinkInterval, blinkCount, false);
        }

        if (!leds.isBusy()) {
            if (useRgbIdle) {
                leds.rgb(dt);
            } else {
                leds.setBase(Leds.Side.LEFT, leftBaseColor);
                leds.setBase(Leds.Side.RIGHT, rightBaseColor);
            }
        }

        leds.update(dt);

        telemetryM.addLine("LED demo controls");
        telemetryM.addLine("A both blink, B right pulse, X left timed hold, Y clear");
        telemetryM.addLine("LB left alert, RB right alert, Dpad left both off hold");
        telemetryM.addLine("Dpad right both pulse, Dpad up toggle RGB idle, Dpad down left no-restore blink");
        telemetryM.addLine("-------------------------");
        telemetryM.addData("dt", dt);
        telemetryM.addData("leds busy", leds.isBusy());
        telemetryM.addData("rgb idle", useRgbIdle);
        telemetryM.addData("left current", leds.getLeft());
        telemetryM.addData("right current", leds.getRight());
        telemetryM.addData("left base", leds.getBaseLeft());
        telemetryM.addData("right base", leds.getBaseRight());
        telemetryM.addData("blink count", blinkCount);
        telemetryM.addData("blink interval", blinkInterval);
        telemetryM.addData("hold duration", holdDuration);
        telemetryM.update(telemetry);
    }
}

// ===================================================
// MOTOR POWER TEST
// ===================================================
@Configurable
class MotorPowerTest extends OpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public MotorPowerTest(MotorConfig motorName) {
        this.motor = motorName;
    }
    private MotorConfig motor;

    private double maxVelocity = 0;
    private static double mult = 1;

    @Override
    public void init() {
        motor.init(hardwareMap);
    }
    @Override
    public void init_loop() {
        if (motor.getMotorUse() == MotorUse.DRIVETRAIN) {
            telemetryM.addLine("WARNING THIS IS A DRIVETRAIN MOTOR");
            telemetryM.addLine("RUN THIS ONLY IF THE ROBOT IS ELEVATED OFF THE GROUND, OR PLACED SIDEWAYS");
        }
        else if (motor.getMotorUse() == MotorUse.MECHANICAL_STOP) {
            telemetryM.addLine("WARNING THIS MOTOR RUNS WITHOUT ENCODER FEEDBACK AND IT MAY DAMAGE MECHANICAL PARTS IF MISUSED");
        }
        else {
            telemetryM.addLine("Init Complete");
        }
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            motor.setDirection(motor.getDirection() == DcMotor.Direction.FORWARD ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        // Set power via triggers
        double power = mult * (gamepad1.right_trigger - gamepad1.left_trigger);
        motor.setPower(power);

        double currentVelocity = motor.getVelocity();
        maxVelocity = Math.max(maxVelocity, Math.abs(currentVelocity));

        telemetryM.addLine("run motor using the triggers");
        telemetryM.addLine("press Y to reverse direction");
        telemetryM.addLine("-----------------------------");
        telemetryM.addData("Power", power);
        telemetryM.addData("Velocity", currentVelocity);
        telemetryM.addData("motor pos", motor.getCurrentPosition());
        telemetryM.addData("current Direction", motor.getDirection().toString());
        telemetryM.addData("current", motor.getCurrent());
        telemetryM.addData("Max Velocity", maxVelocity);
        telemetryM.update(telemetry);
    }
}

class ShooterPowerTest extends OpMode {
    Shooter shooter;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        shooter.init();
        telemetryM.addLine("init");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        shooter.setPower(power);
        telemetryM.addData("power", power);
        telemetryM.addData("current 1", shooter.getCurrent1());
//        telemetryM.addData("current 2", shooter.getCurrent2());
        telemetryM.addData("velocity", shooter.getVelocity());
        telemetryM.update(telemetry);
    }
}
@Configurable
class MotorPositionTest extends OpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    MotorConfig motor;
    ElapsedTime timer = new ElapsedTime();
    public static double targetPosition = 0, kp, ki, kd, ks, kv, ka;
    public MotorPositionTest(MotorConfig motor) {
        this.motor = motor;
    }
    @Override
    public void init() {
        motor.init(hardwareMap);
        kp = motor.kP;
        ki = motor.kI;
        kd = motor.kD;
        ks = motor.kS;
        kv = motor.kV;
        ka = motor.kA;
        PanelsConfigurables.INSTANCE.refreshClass(MotorPositionTest.class);
    }

    @Override
    public void init_loop() {
        if (motor.getMotorUse() == MotorUse.DRIVETRAIN) {
            telemetryM.addLine("WARNING THIS IS A DRIVETRAIN MOTOR");
            telemetryM.addLine("RUN THIS ONLY IF THE ROBOT IS ELEVATED OFF THE GROUND, OR PLACED SIDEWAYS (DRIVETRAIN MOTOR)");
            telemetryM.addLine("THIS IS NOT USEFUL FOR DRIVETRAIN MOTORS");
        }
        else if (motor.getMotorUse() == MotorUse.FREE_SPIN) {
            telemetryM.addLine("THIS MODE IS MADE FOR POSITION CONTROL AND IT IS NOT USEFUL FOR THIS MOTOR");
        }
        else {
            telemetryM.addLine("You can change the PIDF values and position in Panels on"
                    + MotorPositionTest.class.getName());
        }
        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double dt = DebuggerSupport.updateLegacyLoopState(this, timer);
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        motor.setPositionInTicks(targetPosition * motor.getMotorType().getTicksPerDegree());
        motor.updateSimplePositionControl();
        telemetryM.addData("current pos", motor.getCurrentPosition());
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("kp", kp);
        telemetryM.addData("dt", dt);
        telemetryM.addData("target pos", targetPosition);
        telemetryM.update(telemetry);
    }
}
@Configurable
class LimelightTurretAlign extends OpMode {
    ElapsedTime timer = new ElapsedTime();
    MotorConfig motor = RobotConstants.TURRET_CONFIG.copy();
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    Limelight3A limelight;
    public static double maxPower = .2;
    public static double targetPosition = 0, kp, ki, kd, ks;
    @Override
    public void init() {
        motor.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(2);

        kp = motor.kP;
        ki = motor.kI;
        kd = motor.kD;
        ks = motor.kS;
        PanelsConfigurables.INSTANCE.refreshClass(LimelightTurretAlign.class);
    }
    @Override
    public void start() {
        limelight.start();
        timer.reset();
        Log.d("LimelightTurretAlign", "tx,power,time");
    }
    void log() {
        LLResult result = limelight.getLatestResult();
        Log.d("LimelightTurretAlign", String.format("%.2f,%.2f,%.2f",
                result == null ? 0.0 : result.getTx(), motor.getPower(), getRuntime()));
    }
    @Override
    public void loop() {
        double dt = DebuggerSupport.updateLegacyLoopState(this, timer);
        motor.maxPower = maxPower;
        motor.kP = kp; motor.kD = kd;
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetryM.addLine("valid result");
                if (result.getTx() == 0) {
                    motor.manualPositionPIDF(0);
                    telemetryM.addLine("0 power");
                }
                else {
                    motor.manualPositionPIDF(-result.getTx());
                    telemetryM.addLine("running turret");
                }
            }
            else {
                motor.manualPositionPIDF(0);
                telemetryM.addLine("invalid result");
            }
        }
        else {
            telemetryM.addLine("null result");
        }
        telemetryM.addData("kp", motor.kP);
        telemetryM.addData("kd", motor.kD);
        telemetryM.addData("tx", result == null ? 0.0 : result.getTx());
        telemetryM.addData("max power", motor.maxPower);
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("dt", dt);
        telemetryM.update(telemetry);
    }
}


@Configurable
class MotorPIDFVelocityTest extends OpMode {
    boolean runMotor = true;
    public MotorPIDFVelocityTest(MotorConfig motor) {
        this.motor = motor;
    }
    private final MotorConfig motor;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double targetVelocity = 2400, kp, ki, kd, ks, kv, ka;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        motor.init(hardwareMap);
        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
        timer.startTime();
        kp = motor.kP; ki = motor.kI; kd = motor.kD; ks = motor.kS; kv = motor.kV; ka = motor.kA;
    }
    @Override
    public void init_loop() {
        if (motor.getMotorUse() == MotorUse.DRIVETRAIN) {
            telemetryM.addLine("RUN THIS ONLY IF THE ROBOT IS ELEVATED OFF THE GROUND, OR PLACED SIDEWAYS (DRIVETRAIN MOTOR)");
            telemetryM.addLine("THIS IS NOT USEFUL FOR DRIVETRAIN MOTORS");
        } else if (motor.getMotorUse() == MotorUse.MECHANICAL_STOP) {
            telemetryM.addLine("WARNING THIS MOTOR RUNS WITHOUT ENCODER FEEDBACK AND IT MAY DAMAGE MECHANICAL PARTS IF MISUSED");
        } else {
            telemetryM.addLine("You can change the PIDF values and position in Panels on"
                    + MotorPIDFVelocityTest.class.getName());
        }
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double dt = DebuggerSupport.updateLegacyLoopState(this, timer);
        motor.setVelocityTicksPerSecond(targetVelocity);
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        // Set velocity via triggers
        if (gamepad1.bWasPressed()) {
            runMotor ^= true;
        }
        if (runMotor) {
            motor.updateVelocityPIDF();
        }
        else {
            motor.setPower(0);
        }

        telemetryM.addLine("run motor in panels");
        telemetryM.addLine("---------------");
        telemetryM.addData("Current Velocity", motor.getVelocity());
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("current", motor.getCurrent());
        telemetryM.addData("target vel", targetVelocity);
        telemetryM.addData("dt", dt);
        telemetryM.addData("kp", kp);
        telemetryM.addData("motor kp", motor.kP);
        telemetryM.addData("target vel", targetVelocity);
        telemetryM.update(telemetry);
    }
}


class ServoControl extends OpMode{
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    double servoTargetPos = 0;
    String servoName;
    Servo servo;
    public ServoControl(String servoName) {
        this.servoName = servoName;
    }

    @Override
    public void init() {
        servo = hardwareMap.servo.get(servoName);
        servoTargetPos = servo.getPosition();
        telemetryM.addLine("init");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            servo.setDirection(
                    servo.getDirection() == Servo.Direction.FORWARD ?
                            Servo.Direction.REVERSE :
                            Servo.Direction.FORWARD
            );
        } // direction reverser
//        servo.setPosition(gamepad1.right_trigger - gamepad1.left_trigger);
        double analogInput = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(analogInput) < .1) {
            if (gamepad1.dpadLeftWasPressed()) {
                servoTargetPos = servo.getPosition() - .1;
            } else if (gamepad1.dpadRightWasPressed()) {
                servoTargetPos = servo.getPosition() + .1;
            }
        }
        else {
            servoTargetPos = (analogInput + 1.0) / 2.0;
        }
        servoTargetPos = Math.max(0.0, Math.min(1.0, servoTargetPos));
        servo.setPosition(servoTargetPos);
        telemetryM.addData("servo direction", servo.getDirection());
        telemetryM.addData("servo pos", servo.getPosition());
        telemetryM.update(telemetry);
    }
}


class EnableAllServoPwmOpMode extends OpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private int servoCount = 0;

    @Override
    public void init() {
        servoCount = 0;

        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            servo.setPosition(0.5);
            servoCount++;
        }

        telemetryM.addLine("Set every servo to position 0.5");
        telemetryM.addData("servos updated", servoCount);
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        telemetryM.addLine("All servos were commanded to 0.5 during init");
        telemetryM.addData("servos updated", servoCount);
        telemetryM.update(telemetry);
    }
}

class FlickerAnalogControl extends OpMode{
    Flickers flickers = new Flickers();
    @Override
    public void init() {
        flickers.init(hardwareMap);
    }

    @Override
    public void loop() {
        double leftFlickerPos = .5 + gamepad1.left_trigger * .5f;
        double rightFlickerPos = .5 + gamepad1.right_trigger * .5f;
        flickers.setLeftFlickerPos(leftFlickerPos);
        flickers.setRightFlickerPos(rightFlickerPos);
        telemetry.addData("left flicker pos", leftFlickerPos);
        telemetry.addData("right flicker pos", rightFlickerPos);
        telemetry.update();
    }
}

class HangControl extends OpMode {
    Hang hang = new Hang();
    TelemetryManager telemetryM  = PanelsTelemetry.INSTANCE.getTelemetry();
    static double power = 1;
    static double degrees = 90;
    @Override
    public void init() {
        hang.init(hardwareMap);
    }

    @Override
    public void loop() {
        hang.update(power, (int) ((gamepad1.right_trigger - gamepad1.left_trigger) * degrees));
        telemetryM.update(telemetry);
    }
}

@Configurable
class KeCharacterizationOpMode extends OpMode {
    Shooter shooter;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    static int SAMPLES = 10;
    static double maxPower = 1;
    double[] powerLevels = new double[SAMPLES];
    {
        for (int i = 1; i <= SAMPLES; i++) {
            powerLevels[i-1] = i * maxPower * (1.0 / (SAMPLES));
        }
    }
    int index = 0;
    double lastVelocity = 0.0;
    long stableStartTime = 0;
    boolean steady = false;
    String TAG = "KeChar";
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        Log.d(TAG, "velocity,applied_voltage");
        shooter = new Shooter(hardwareMap);
        shooter.init();
    }
    @Override
    public void init_loop() {
        telemetryM.addLine("Ke Characterization Ready");
//        telemetryM.addLine((shooter.getMotorUse() == MotorUse.DRIVETRAIN) ? "WARNING: RUN THIS ONLY IF THE ROBOT IS ELEVATED OFF THE GROUND, OR PLACED SIDEWAYS":"");
//        telemetryM.addLine((shooter.getMotorUse() == MotorUse.MECHANICAL_STOP) ? "WARNING: THIS MOTOR RUNS WITHOUT ENCODER FEEDBACK AND IT MAY DAMAGE MECHANICAL PARTS IF MISUSED":"");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        index = 0;
        applyPower();
        lastVelocity = shooter.getVelocity();
        timer.reset();
    }

    @Override
    public void loop() {
        if (index >= powerLevels.length) {
            shooter.setPower(0);
            telemetryM.addLine("Done");
            telemetryM.addData("LogCat tag", TAG);
            return;
        }

        double currentVelocity = shooter.getVelocity();
        double dt = Math.max(timer.seconds(), 1e-6);
        double accel = (currentVelocity - lastVelocity) / dt; // ~20ms loop

        if (gamepad1.yWasPressed()) {
            logPoint();
            index++;
            applyPower();
        }

        lastVelocity = currentVelocity;

        telemetryM.addData("Index", index);
        telemetryM.addData("Velocity (ticks/s)", currentVelocity);
        telemetryM.addData("Accel (ticks/s^2)", accel);

        telemetryM.update(telemetry);
        timer.reset();
    }

    private void applyPower() {
        if (index < powerLevels.length) {
            shooter.setPower(powerLevels[index]);
        }
    }

    private void logPoint() {
        double velocity = shooter.getVelocity();
        double batteryVoltage = getBatteryVoltage();
        double appliedVoltage = powerLevels[index] * batteryVoltage;

        telemetryM.addLine("=== DATA POINT ===");
        telemetryM.addData("Power", powerLevels[index]);
        telemetryM.addData("Velocity (ticks/s)", velocity);
        telemetryM.addData("Battery Voltage (V)", batteryVoltage);
        telemetryM.addData("Applied Voltage (V)", appliedVoltage);
        telemetryM.update();
        Log.d(TAG, String.format("%.3f,%.3f",
                velocity, appliedVoltage)
        );

    }

    private double getBatteryVoltage() {
        return DebuggerSupport.getBatteryVoltage(hardwareMap);
    }
}

@Configurable
class KaTestOpMode extends OpMode {
    MotorConfig motor;
//    VoltageSensorReadout voltageSensor;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Iterable<VoltageSensor> voltageSensors;
    static double degreesOffset = 30;
    private double lastTime = 0.0;
    private double timer;
    private static double kp, ki, kd, ks, kv, ka, maxVel, maxAcc;
    private int direction = 1;
    static double maxPower = 1;
    static double loopTime = 2;
    public KaTestOpMode(MotorConfig motor) {
        this.motor = motor;
    }
    static boolean logPoints = false;
    public void log() {
        double velocity = motor.getVelocity();
        double appliedVoltage = motor.getPower() * getBatteryVoltage();
        double current = motor.getCurrent();
        double power = motor.getPower();
        double position = motor.getCurrentPosition();
        double xref = motor.getxRef();
        double vref = motor.getvRef();
        double aref = motor.getaRef();
        double targetPos = motor.getTargetPositionTicks();


        Log.d("KaTest", String.format("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                velocity, appliedVoltage, current, power, position, xref, vref, aref, targetPos, getRuntime())
        );
    }

    @Override
    public void init() {
//        voltageSensor.init(hardwareMap);
        kp = motor.kP; ki = motor.kI; kd = motor.kD; ks = motor.kS; kv = motor.kV; ka = motor.kA;
        maxPower = motor.maxPower;
        maxVel = motor.maxVelocity;
        maxAcc = motor.maxAcceleration;
        voltageSensors = hardwareMap.voltageSensor;
        Log.d("KaTest", "velocity,applied_voltage,current,power,position,xref,vref,aref,targetPos,time");
        PanelsConfigurables.INSTANCE.refreshClass(this);
        motor.init(hardwareMap);
    }
    @Override
    public void init_loop() {
//        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void start() {
        lastTime = getRuntime();
        timer = 0.0;
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0) return;

        timer += dt;
        MotorConfig.setDt(dt);
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        motor.maxAcceleration = maxAcc;
        motor.maxVelocity = maxVel;
        motor.maxPower = maxPower;
        if (timer >= loopTime) {
            direction *= -1;
            timer = 0;
        }
        motor.setPositionInDegrees(degreesOffset * direction);
        motor.updatePositionProfiledPIDF();
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("max power", motor.maxPower);
        telemetryM.addData("velocity", motor.getVelocity());
        telemetryM.addData("ref vel", motor.getvRef());
        telemetryM.addData("position", motor.getCurrentPosition());
        telemetryM.addData("ref pos", motor.getxRef());
        telemetryM.addData("target pos", degreesOffset * direction);
        telemetryM.addData("ref a", motor.getaRef());
        telemetryM.addData("current", motor.getCurrent());
        telemetryM.addData("dt", dt);
        telemetryM.addData("timer", timer);
        telemetryM.addData("ks", ks);
        telemetryM.addData("ks motor", motor.kS);
        telemetryM.update(telemetry);
        if (logPoints) log();
    }

    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage < Double.POSITIVE_INFINITY ? minVoltage : 12.0;
    }
}

@Configurable
class TurretStickTeleOp extends OpMode {
    private final MotorConfig motor;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Turret turret;

    public TurretStickTeleOp(MotorConfig motor) {
        this.motor = motor;
    }

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(Turret.class);
        turret = new Turret(hardwareMap);
        turret.init();
    }

    @Override
    public void start() {
        turret.start();
    }

    @Override
    public void loop() {
        double stick = gamepad1.right_stick_x;
        double targetRadians = stick >= 0
                ? stick * RobotConstants.TURRET_MAX_ANGLE_RADIANS
                : stick * Math.abs(RobotConstants.TURRET_MIN_ANGLE_RADIANS);

        turret.setAngleRadians(targetRadians);
        turret.loop();

        telemetryM.addLine("Use gamepad1 right stick X to command the turret");
        telemetryM.addData("stick", stick);
        telemetryM.addData("target radians", targetRadians);
        telemetryM.addData("target degrees", Math.toDegrees(targetRadians));
        telemetryM.addData("min degrees", Math.toDegrees(RobotConstants.TURRET_MIN_ANGLE_RADIANS));
        telemetryM.addData("max degrees", Math.toDegrees(RobotConstants.TURRET_MAX_ANGLE_RADIANS));
        telemetryM.update(telemetry);
    }
}

class VoltageSensorReadoutOpMode extends OpMode {

    private Iterable<VoltageSensor> voltageSensors;

    @Override
    public void init() {
        // Get all voltage sensors from the hardware map
        voltageSensors = hardwareMap.voltageSensor;

        telemetry.addLine("Voltage sensors initialized");
        telemetry.update();
    }

    public void init_loop() {
        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {

        int index = 0;
        double minVoltage = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();

            telemetry.addLine("VoltageSensor[" + index + "]");
            telemetry.addData("  Device Name", sensor.getDeviceName());
            telemetry.addData("  Connection", sensor.getConnectionInfo());
            telemetry.addData("  Voltage (V)", "%.2f", voltage);

            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }

            index++;
        }

        telemetry.addLine("-------------------------");

        if (minVoltage < Double.POSITIVE_INFINITY) {
            telemetry.addData("Robot Battery Voltage (V)", "%.2f", minVoltage);
        } else {
            telemetry.addLine("No valid voltage readings");
        }

        telemetry.update();
    }
}

@Configurable
class ColorReadoutOpMode extends OpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Intake intake;
    private Gate gate = new Gate();
    private ColorSensors colors = new ColorSensors();
    static boolean stopIntakeWhenFull = true;

    @Override
    public void init() {
        gate.init(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.init();
        colors.init(hardwareMap);

        telemetryM.addLine("Color readout ready");
        telemetryM.addLine("Use triggers to control intake");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        colors.update();
        intake.setCurrentState(
                colors.isFull() && stopIntakeWhenFull
                        ? Intake.IntakeState.STOP
                        : Math.abs(intakePower) < 0.05
                        ? Intake.IntakeState.STOP
                        : intakePower > 0
                        ? Intake.IntakeState.INTAKE
                        : Intake.IntakeState.OUTTAKE
        );
        intake.update();
        if (gamepad1.rightBumperWasPressed()) gate.changeState();

        telemetryM.addData("intake power", intakePower);
        telemetryM.addData("intake current", intake.getCurrent());
        telemetryM.addLine("-------------------------");
        telemetryM.addData("is robot full", colors.isFull());
        telemetryM.addData("color1 distance (cm)", colors.getColor1());
        telemetryM.addData("color1 detected", colors.is1Detected());
        telemetryM.addData("color2 distance (cm)", colors.getColor2());
        telemetryM.addData("color2 detected", colors.is2Detected());
        telemetryM.addData("color3 distance (cm)", colors.getColor3());
        telemetryM.addData("color3 detected", colors.is3Detected());
        telemetryM.update(telemetry);
    }
}

@Configurable
class RampPowerOpMode extends OpMode {
    public RampPowerOpMode(MotorConfig motor) {
        this.motor = motor;
    }
    static double acceleration = 1.0; // change in power per second
    MotorConfig motor;
    double lastTime = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private double rampPower(double current, double target, double dt) {
        double maxPowerChange = acceleration * dt;
        double diff = target - current;
        if (Math.abs(diff) > maxPowerChange) {
            diff = Math.signum(diff) * maxPowerChange;
        }
        return current + diff;
    }
    @Override
    public void init() {
        motor.init(hardwareMap);
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double targetPower = gamepad1.right_trigger - gamepad1.left_trigger;
        double currentPower = motor.getPower();
        double newPower = rampPower(currentPower, targetPower, dt);
        motor.setPower(newPower);
        telemetryM.addData("Target Power", targetPower);
        telemetryM.update(telemetry);
        lastTime = currentTime;
    }
}

    @Configurable
    class TestThroughput extends OpMode {
        static double velocity = 1300;
        static boolean runWithVel = true;
        private static final double TRIGGER_PRESS_THRESHOLD = 0.25;
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Intake intake;
        Shooter shooter;
        Gate gate;
        boolean lastRightTriggerPressed;
        @Override
        public void init() {
            runWithVel = true;
            intake = new Intake(hardwareMap);
            intake.init();
            shooter = new Shooter(hardwareMap);
            shooter.init();
            gate = new Gate();
            gate.init(hardwareMap);
            telemetryM.addLine("init complete");
            telemetryM.update(telemetry);
            Log.d("ThroughputTest", "isUsingController,shooter_velocity,shooter_power,time,intake_current,shooter_current,shooter_target");
        }

        @Override
        public void loop() {
            boolean rightTriggerPressed = gamepad1.right_trigger > TRIGGER_PRESS_THRESHOLD;
            boolean trigger = rightTriggerPressed && !lastRightTriggerPressed;
            lastRightTriggerPressed = rightTriggerPressed;
            if (trigger && !gate.isActivated()) {
                gate.activate();
            }
            else if (trigger && gate.isActivated()) {
                gate.deactivate();
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) log();
            if (gamepad1.dpadUpWasPressed()) shooter.changeState();
            if (gamepad1.yWasPressed()) runWithVel ^= true;
            if (gamepad1.aWasPressed()) intake.setCurrentState(Intake.IntakeState.INTAKE);
            else if (gamepad1.bWasPressed()) intake.setCurrentState(Intake.IntakeState.STOP);

            Shooter.targetVelocity = velocity;
            if (runWithVel) {
                shooter.run(true);
                shooter.update();
            }
            else {
                shooter.run(false);
                double ffPower = (RobotConstants.SHOOTER_CONFIG.kS * Math.signum(velocity)
                        + RobotConstants.SHOOTER_CONFIG.kV * velocity) / 12.0;
                shooter.setPower(ffPower);
            }

            intake.update();
            telemetryM.addLine("running with " + (runWithVel ? "velocity":"open loop power"));
            telemetryM.addData("shooter enabled", shooter.getRun());
            telemetryM.addData("shooter velocity", shooter.getVelocity());
            telemetryM.addData("shooter power", shooter.getPower());
            telemetryM.update(telemetry);
        }
        public void log() {
            Log.d("ThroughputTest", String.format("%b,%.2f,%.2f,%.2f,%.2f,%.2f,%f", runWithVel, shooter.getVelocity(), shooter.getPower(), getRuntime(), intake.getCurrent(), shooter.getCurrent1(), Shooter.targetVelocity));
        }
    }
@Configurable
class CollectData extends OpMode {
    Pose startPose = new Pose(72, 72, 0);
    Follower follower;
    Gate gate;
    Intake intake;
    Shooter shooter;
    Turret turret;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    final String tag = "collectData";
    boolean runShooter = true;
    static double shooterTarget = 1000;
    double newTime, dt, oldTime;

    @Override
    public void init() {
        shooterTarget = 1000;
        gate = new Gate();
        gate.init(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.init();
        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(false);
        turret = new Turret(hardwareMap);
        turret.init();
        follower = PPConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        telemetryM.addLine("init complete");
        telemetryM.update(telemetry);
        Log.d(tag, "is_successful,pose,turret_angle,shooter_velocity,hood_angle,velocity");
    }

    @Override
    public void start() {
        oldTime = getRuntime();
    }

    @Override
    public void loop() {
        newTime = getRuntime();
        dt = newTime - oldTime;
        oldTime = newTime;
        if (dt < 0) {
            dt = 0;
        }
        if (gamepad1.rightBumperWasPressed()) gate.changeState();
        shooter.setHoodAngle(shooter.getHoodAngle() + .5 * dt * (gamepad1.right_trigger-gamepad1.left_trigger));
        if (gamepad1.xWasPressed()) {
            shooter.changeState();
        }
        if (gamepad1.yWasPressed()) {
            intake.setCurrentState(
                    intake.getCurrentState() == Intake.IntakeState.INTAKE
                            ? Intake.IntakeState.STOP
                            : Intake.IntakeState.INTAKE
            );
        }
        Shooter.targetVelocity = shooterTarget;
        intake.update();
        shooter.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        turret.lookToGoal(follower.getPose(), false);
        turret.loop();
        double turretPos = turret.getAngleToGoal();
        double shooterVel = shooter.getVelocity();
        double hoodAngle = shooter.getHoodAngle();
        String followerPose = follower.getPose().toString();
        String vel = follower.getVelocity().toString();
        if (gamepad1.aWasPressed()) Log.d(
                tag, String.format(
                        "%b,%s,%f,%f,%f,%s",
                        true, followerPose, turretPos, shooterVel, hoodAngle, vel
                )
        );
        else if (gamepad1.bWasPressed()) Log.d(
                tag, String.format(
                        "%b,%s,%f,%f,%f,%s",
                        false, followerPose, turretPos, shooterVel, hoodAngle, vel
                )
        );
        telemetryM.addData("follower pose", follower.getPose().toString());
        telemetryM.addData("turret angle", turretPos);
        telemetryM.addData("shooter velocity", shooterVel);
        telemetryM.addData("intake state", intake.getCurrentState());
        telemetryM.update(telemetry);
    }
}

final class DebuggerSupport {
    private DebuggerSupport() {}

    static double getBatteryVoltage(OpMode opMode) {
        return getBatteryVoltage(opMode.hardwareMap);
    }

    static double getBatteryVoltage(HardwareMap hardwareMap) {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage < Double.POSITIVE_INFINITY ? minVoltage : 12.0;
    }

    static double updateLegacyLoopState(OpMode opMode, ElapsedTime timer) {
        double dt = timer.seconds();
        MotorConfig.setDt(dt);
        MotorConfig.setBatteryVoltage(getBatteryVoltage(opMode));
        timer.reset();
        return dt;
    }
}

