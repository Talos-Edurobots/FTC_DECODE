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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorUse;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;


@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {
    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", ne -> {
                ne.add("Run Intake",  () -> new MotorPowerTest(RobotConstants.INTAKE_CONFIG ));
                ne.add("Run Shooter", () -> new MotorPowerTest(RobotConstants.SHOOTER_CONFIG));
                ne.add("run turret",  () -> new MotorPowerTest(RobotConstants.TURRET_CONFIG ));
                ne.add("run hang", () -> new MotorPowerTest(RobotConstants.HANG_CONFIG));
                ne.add("Run Left Front Drive",  () -> new MotorPowerTest(RobotConstants.LEFT_FRONT_CONFIG ));
                ne.add("Run Right Front Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_FRONT_CONFIG));
                ne.add("Run Left Back Drive",   () -> new MotorPowerTest(RobotConstants.LEFT_BACK_CONFIG  ));
                ne.add("Run Right Back Drive",  () -> new MotorPowerTest(RobotConstants.RIGHT_BACK_CONFIG ));
            });
            s.folder("Velocity Control", vc -> {
                vc.add("Run Shooter Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.SHOOTER_CONFIG));
                vc.add("Run Intake Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.INTAKE_CONFIG));
            });
            s.folder("servo control", sc -> {
                sc.add("right servo", () -> new ServoControl(RobotConstants.RIGHT_SERVO_NAME));
                sc.add("left servo", () -> new ServoControl(RobotConstants.LEFT_SERVO_NAME));
                sc.add("right flicker", () -> new ServoControl(RobotConstants.RIGHT_FLICKER_NAME));
                sc.add("left flicker", () -> new ServoControl(RobotConstants.LEFT_FLICKER_NAME));
            });
            s.folder("position control", pc -> {
                pc.add("turret position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG));
                pc.add("turret ka test", () -> new KaTestOpMode(RobotConstants.TURRET_CONFIG));
                pc.add("turret stick teleop", () -> new TurretStickTeleOp(RobotConstants.TURRET_CONFIG));
            });
            s.folder("ke characterization", kect -> {
                kect.add("shooter ke", () -> new KeCharacterizationOpMode(RobotConstants.SHOOTER_CONFIG));
                kect.add("intake ke", () -> new KeCharacterizationOpMode(RobotConstants.INTAKE_CONFIG));
                kect.add("turret ke", () -> new KeCharacterizationOpMode(RobotConstants.TURRET_CONFIG));
            });
            s.folder("high level", hlt -> {
                hlt.add("flicker analog control", FlickerAnalogControl::new);
                hlt.add("voltage sensor readout", VoltageSensorReadoutOpMode::new);
                hlt.add("color readout", ColorReadoutOpMode::new);
//                hlt.add("turret position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG));
                hlt.add("hang control", HangControl::new);
                hlt.add("turret simple pidf with turret", LimelightTurretAlign::new);
                hlt.add("through put test", TestThoughPut::new);
                hlt.add("collect data", CollectData::new);
            });
        });
    }

    @Override
    public void onSelect() {}
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
        MotorConfig.setDt(timer.seconds());
        timer.reset();
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        motor.setPositionInTicks(targetPosition * motor.getMotorType().getTicksPerDegree());
        motor.updateSimplePositionControl();
        telemetryM.addData("current pos", motor.getCurrentPosition());
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("kp", kp);
        telemetryM.addData("target pos", targetPosition);
        telemetryM.update(telemetry);
    }
}
@Configurable
class LimelightTurretAlign extends OpMode {
    ElapsedTime timer = new ElapsedTime();
    MotorConfig motor = RobotConstants.TURRET_CONFIG;
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
        PanelsConfigurables.INSTANCE.refreshClass(MotorPositionTest.class);
    }
    @Override
    public void start() {
        limelight.start();
        timer.reset();
        Log.d("LimelightTurretAlign", "tx,power,time");
    }
    void log() {
        Log.d("LimelightTurretAlign", String.format("%.2f,%.2f,%.2f",
                limelight.getLatestResult().getTx(), motor.getPower(), getRuntime()));
    }
    @Override
    public void loop() {
        MotorConfig.setDt(timer.seconds());
        timer.reset();
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
        telemetryM.addData("tx", result.getTx());
        telemetryM.addData("max power", motor.maxPower);
        telemetryM.addData("power", motor.getPower());
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
        MotorConfig.setDt(timer.seconds());
        MotorConfig.setBatteryVoltage(12);
        timer.reset();
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
        telemetryM.addData("dt", timer.seconds());
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
            servo.setPosition(servoTargetPos);
        }
        else {
            servo.setPosition(analogInput);
        }
//        servoTargetPos = Math.min(1.0, Math.max(0.0, servoTargetPos));
//        servoTargetPos = gamepad1.right_trigger - gamepad1.left_trigger;
        servo.setPosition(servoTargetPos);
        telemetryM.addData("servo direction", servo.getDirection());
        telemetryM.addData("servo pos", servo.getPosition());
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
    public KeCharacterizationOpMode(MotorConfig motor) {
        this.motor = motor;
    }
    MotorConfig motor;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    static int SAMPLES = 10;
    static double maxPower = .5;
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
        motor.init(hardwareMap);
        Log.d(TAG, "velocity,applied_voltage");
    }
    @Override
    public void init_loop() {
        telemetryM.addLine("Ke Characterization Ready");
        telemetryM.addLine((motor.getMotorUse() == MotorUse.DRIVETRAIN) ? "WARNING: RUN THIS ONLY IF THE ROBOT IS ELEVATED OFF THE GROUND, OR PLACED SIDEWAYS":"");
        telemetryM.addLine((motor.getMotorUse() == MotorUse.MECHANICAL_STOP) ? "WARNING: THIS MOTOR RUNS WITHOUT ENCODER FEEDBACK AND IT MAY DAMAGE MECHANICAL PARTS IF MISUSED":"");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        index = 0;
        applyPower();
        lastVelocity = motor.getVelocity();
        stableStartTime = System.currentTimeMillis();
        timer.reset();
    }

    @Override
    public void loop() {
        if (index >= powerLevels.length) {
            motor.setPower(0);
            telemetryM.addLine("Done");
            telemetryM.addData("LogCat tag", TAG);
            return;
        }

        double currentVelocity = motor.getVelocity();
        double accel = (currentVelocity - lastVelocity) / timer.seconds(); // ~20ms loop

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
            motor.setPower(powerLevels[index]);
        }
    }

    private void logPoint() {
        double velocity = motor.getVelocity();
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
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) minVoltage = Math.min(minVoltage, v);
        }
        return minVoltage;
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
    private double lastTime = 0.0;
    private static double kp, ki, kd, ks, kv, ka, maxVel, maxAcc;
    static double maxPower = 1;

    public TurretStickTeleOp(MotorConfig motor) {
        this.motor = motor;
    }

    @Override
    public void init() {
        kp = motor.kP;
        ki = motor.kI;
        kd = motor.kD;
        ks = motor.kS;
        kv = motor.kV;
        ka = motor.kA;
        maxPower = motor.maxPower;
        maxVel = motor.maxVelocity;
        maxAcc = motor.maxAcceleration;
        PanelsConfigurables.INSTANCE.refreshClass(this);
        motor.init(hardwareMap);
    }

    @Override
    public void start() {
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0) return;

        double stick = gamepad1.right_stick_x;
        double targetTicks = stick >= 0
                ? stick * motor.getMaxAngleTicks()
                : stick * Math.abs(motor.getMinAngleTicks());
//                : stick * motor.getMinAngleTicks();

        MotorConfig.setDt(dt);
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        motor.maxAcceleration = maxAcc;
        motor.maxVelocity = maxVel;
        motor.maxPower = maxPower;
        motor.setPositionInTicks(targetTicks);
        motor.updatePositionProfiledPIDF();

        telemetryM.addLine("Use gamepad1 right stick X to command the turret");
        telemetryM.addData("stick", stick);
        telemetryM.addData("target ticks", targetTicks);
        telemetryM.addData("min ticks", motor.getMinAngleTicks());
        telemetryM.addData("max ticks", motor.getMaxAngleTicks());
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("velocity", motor.getVelocity());
        telemetryM.addData("position", motor.getCurrentPosition());
        telemetryM.addData("ref pos", motor.getxRef());
        telemetryM.addData("ref vel", motor.getvRef());
        telemetryM.addData("current", motor.getCurrent());
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

class ColorReadoutOpMode extends OpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final MotorConfig intakeMotor = RobotConstants.INTAKE_CONFIG;
    private Gate gate = new Gate();
    private ColorSensors colors = new ColorSensors();

    @Override
    public void init() {
        gate.init(hardwareMap);
        intakeMotor.init(hardwareMap);
        colors.init(hardwareMap);

        telemetryM.addLine("Color readout ready");
        telemetryM.addLine("Use triggers to control intake");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        intakeMotor.setPower(colors.isFull() ? 0 : intakePower);
        colors.update();
        if (gamepad1.rightBumperWasPressed()) gate.changeState();

        telemetryM.addData("intake power", intakePower);
        telemetryM.addData("intake current", intakeMotor.getCurrent());
        telemetryM.addLine("-------------------------");
        telemetryM.addData("is robot full", colors.isFull());
        telemetryM.addData("color1 distance (cm)", colors.getColor1());
        telemetryM.addData("is detected", colors.is1Detected());
        telemetryM.addData("color2 distance (cm)", colors.getColor2());
        telemetryM.addData("is detected", colors.is2Detected());
        telemetryM.addData("color3 distance (cm)", colors.getColor3());
        telemetryM.addData("is detected", colors.is3Detected());
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
        double maxPowerChange = RobotConstants.DrivetrainMaxAcceleration * dt;
        double diff = target - current;
        if (Math.abs(diff) > maxPowerChange) {
            diff = Math.signum(diff) * maxPowerChange;
        }
        return current + diff;
    }
    @Override
    public void init() {
        motor.init(hardwareMap);
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
    class TestThoughPut extends OpMode {
        static double velocity = 1300;
        static boolean runWIthVel = true;
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Intake intake;
        Shooter shooter;
        Gate gate;
        @Override
        public void init() {
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
            boolean trigger = gamepad1.rightTriggerWasPressed();
            if (trigger && !gate.isActivated()) {
                gate.activate();
            }
            else if (trigger && gate.isActivated()) {
                gate.deactivate();
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) log();
            if (gamepad1.dpadUpWasPressed()) shooter.changeState();
            if (gamepad1.yWasPressed()) runWIthVel ^= true;
            if (gamepad1.aWasPressed()) intake.setCurrentState(Intake.IntakeState.INTAKE);
            else if (gamepad1.bWasPressed()) intake.setCurrentState(Intake.IntakeState.STOP);

            Shooter.targetVelocity = velocity;
            if (runWIthVel) {
                shooter.run(true);
                shooter.update();
            }
            else {
                shooter.run(false);
                double ffPower = (shooter.getInstance().kS * Math.signum(velocity)
                        + shooter.getInstance().kV * velocity) / 12.0;
                shooter.setPower(ffPower);
            }

            intake.update();
            telemetryM.addLine("running with " + (runWIthVel ? "velocity":"open loop power"));
            telemetryM.addData("shooter enabled", shooter.getRun());
            telemetryM.addData("shooter velocity", shooter.getVelocity());
            telemetryM.addData("shooter power", shooter.getInstance().getPower());
            telemetryM.update(telemetry);
        }
        public void log() {
            Log.d("ThroughputTest", String.format("%b,%.2f,%.2f,%.2f,%.2f,%.2f,%f", runWIthVel, shooter.getVelocity(), shooter.getInstance().getPower(), getRuntime(), intake.getCurrent(), shooter.getCurrent(), Shooter.targetVelocity));
        }
    }
@Configurable
class CollectData extends OpMode {
    Pose startPose = new Pose(72, 72, 0);
    Follower follower;
    Gate gate;
    Intake intake;
    Shooter shooter;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    MotorConfig turret = RobotConstants.TURRET_CONFIG;
    final String tag = "collectData";
    boolean runShooter = true;
    static double shooterTarget = 1000;
    double newTime, dt, oldTime;

    @Override
    public void init() {
        gate = new Gate();
        gate.init(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.init();
        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(false);
        follower = PPConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        turret.init(hardwareMap);
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
        double turretPos = turret.getCurrentPosition()/turret.getMotorType().getTicksPerRadian();
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

