package org.firstinspires.ftc.teamcode.pedroPathing.main;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;


@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {
    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", ne -> {
                ne.add("Run Intake",  () -> new MotorPowerTest(RobotConstants.INTAKE_CONFIG ));
                ne.add("Run Shooter", () -> new MotorPowerTest(RobotConstants.SHOOTER_CONFIG));
                ne.add("run turret",  () -> new MotorPowerTest(RobotConstants.TURRET_CONFIG ));
                ne.add("Run Left Front Drive",  () -> new MotorPowerTest(RobotConstants.LEFT_FRONT_CONFIG ));
                ne.add("Run Right Front Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_FRONT_CONFIG));
                ne.add("Run Left Back Drive",   () -> new MotorPowerTest(RobotConstants.LEFT_BACK_CONFIG  ));
                ne.add("Run Right Back Drive",  () -> new MotorPowerTest(RobotConstants.RIGHT_BACK_CONFIG ));
            });
            s.folder("Velocity Control", vc -> {;
                vc.add("Run Shooter Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.SHOOTER_NAME));
                vc.add("Run Intake Velocity", () -> new MotorPIDFVelocityTest(RobotConstants.INTAKE_NAME));
            });
            s.folder("servo control", sc -> {
                sc.add("right servo", () -> new ServoControl(RobotConstants.RIGHT_SERVO_NAME));
                sc.add("left servo", () -> new ServoControl(RobotConstants.LEFT_SERVO_NAME));
                sc.add("right flicker", () -> new ServoControl(RobotConstants.RIGHT_FLICKER_NAME));
                sc.add("left flicker", () -> new ServoControl(RobotConstants.LEFT_FLICKER_NAME));
            });
            s.folder("high level", hlt -> {
//                hlt.add("shooter with servo", () -> new ShooterOpMode(
//                        new MotorPIDFVelocityTest(RobotConstants.SHOOTER_NAME),
//                        new ServoControl(RobotConstants.LEFT_SERVO_NAME)
//                ));
//                hlt.add("shooter ke", KeCharacterizationOpMode::new);
                hlt.add("flicker analog control", FlickerAnalogControl::new);
                hlt.add("voltage sensor readout", VoltageSensorReadoutOpMode::new);
                hlt.add("motor position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG));
            });
        });
    }

    @Override
    public void onSelect() {}
}

// ===================================================
// MOTOR POWER TEST
// ===================================================
class MotorPowerTest extends OpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public MotorPowerTest(MotorConfig motorName) {
        this.motor = motorName;
    }
    private MotorConfig motor;

    private double maxVelocity = 0;

    @Override
    public void init() {

    }
    @Override
    public void init_loop() {
        telemetryM.addLine("WARNING THIS MOTOR RUNS WITHOUT ENCODER FEEDBACK AND IT MAY DAMAGE MECHANICAL PARTS IF MISUSED");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            motor.setDirection(motor.getDirection() == DcMotor.Direction.FORWARD ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        // Set power via triggers
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        motor.setPower(power);

        double currentVelocity = motor.getVelocity();
        maxVelocity = Math.max(maxVelocity, Math.abs(currentVelocity));

        telemetryM.addLine("run motor using the triggers");
        telemetryM.addLine("press Y to reverse direction");
        telemetryM.addLine("-----------------------------");
        telemetryM.addData("Power", power);
        telemetryM.addData("Velocity", currentVelocity);
        telemetryM.addData("motor pos", motor.getVelocity());
        telemetryM.addData("current Direction", motor.getDirection().toString());
        telemetryM.addData("current", motor.getCurrent());
        telemetryM.addData("Max Velocity", maxVelocity);
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
    public static double targetVelocity = 2400, k_p = 0.01, k_i = 0, k_d = 0, k_s = 0.02, k_u = 0.0004;
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
        timer.startTime();
    }
    @Override
    public void init_loop() {
        telemetryM.addLine("Init Complete");
        telemetryM.addLine("You can change the PIDF values and position in Panels on"
                + MotorPIDFVelocityTest.class.getName());
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {

        timer.reset();
        // Set velocity via triggers
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        if (gamepad1.bWasPressed()) {
            runMotor ^= true;
        }
        if (runMotor) {
            motor.setPower(
                    k_p * error +
                            k_i * integralSum +
                            k_d * derivative +
                            k_s * Math.signum(targetVelocity) +
                            k_u * targetVelocity
            );
        }
        else {
            motor.setPower(0);
        }

        telemetryM.addLine("run motor in panels");
        telemetryM.addLine("---------------");
        telemetryM.addData("Current Velocity", currentVelocity);
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("current", motor.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("target vel", targetVelocity);
//        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM.update(telemetry);
        lastError = error;
    }
}

@Configurable
class MotorPositionTest extends OpMode {

    public static double maxPower = 1.0;
    boolean runMotor = true;

    private final MotorConfig motor;

    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    // PID constants
    public static double targetPosition = 0;   // encoder ticks
    public static double k_p = 0.005;
    public static double k_i = 0.0;
    public static double k_d = 0.0005;
    public static double k_s = 0.05;            // static friction compensation

    public static DcMotorSimple.Direction direction =
            DcMotorSimple.Direction.FORWARD;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public MotorPositionTest(MotorConfig motor) {
        this.motor = motor;
    }

    @Override
    public void init() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(direction);

        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);

        timer.reset();
    }

    @Override
    public void init_loop() {
        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {

        double dt = timer.seconds();
        timer.reset();

        int currentPosition = motor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        if (gamepad1.bWasPressed()) {
            runMotor ^= true;
        }

        double output = 0;

        if (runMotor) {
            output =
                    k_p * error +
                            k_i * integralSum +
                            k_d * derivative +
                            k_s * Math.signum(error);

            output = Math.max(-maxPower, Math.min(maxPower, output));
            motor.setPower(output);
        } else {
            motor.setPower(0);
        }

        telemetryM.addLine("Position PID Test");
        telemetryM.addLine("----------------");
        telemetryM.addData("Target Position", targetPosition);
        telemetryM.addData("Current Position", currentPosition);
        telemetryM.addData("Error", error);
        telemetryM.addData("Power", output);
        telemetryM.addData("Current (A)", motor.getCurrent());

        telemetryM.update(telemetry);

        lastError = error;
    }
}

@Configurable
class MotorPositionTestProfiled extends OpMode {

    /* ---------------- Configuration ---------------- */

    public static double maxPower = 1.0;

    // Final goal (encoder ticks)
    public static double targetPosition = 2000;

    // Motion profile limits
    public static double maxVelocity = 1500;      // ticks/s
    public static double maxAcceleration = 3000;  // ticks/s^2

    // PID (position error)
    public static double kP = 0.005;
    public static double kD = 0.0005;

    // Feedforward (characterized)
    public static double kV = 0.0008;  // V per (tick/s)
    public static double kA = 0.0003;  // V per (tick/s^2)
    public static double kS = 0.05;    // static friction (V)

    public static DcMotorSimple.Direction direction =
            DcMotorSimple.Direction.FORWARD;

    /* ---------------- Hardware ---------------- */

    private final String motorName;
    private DcMotorEx motor;

    private TelemetryManager telemetryM =
            PanelsTelemetry.INSTANCE.getTelemetry();

    /* ---------------- State ---------------- */

    private double lastError = 0;
    private double lastPosition = 0;
    private double lastVelocityRef = 0;

    private double profilePosition = 0;
    private double profileVelocity = 0;

    private ElapsedTime timer = new ElapsedTime();

    public MotorPositionTestProfiled(String motorName) {
        this.motorName = motorName;
    }

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(direction);

        timer.reset();
        lastPosition = 0;

        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {

        /* -------- Timing -------- */
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) return;

        /* -------- Measured state -------- */
        double currentPosition = motor.getCurrentPosition();
        double currentVelocity =
                (currentPosition - lastPosition) / dt;
        lastPosition = currentPosition;

        /* -------- Motion Profile -------- */
        double errorToGoal = targetPosition - profilePosition;

        // Desired acceleration
        double desiredAccel =
                Math.signum(errorToGoal) * maxAcceleration;

        // Deceleration check
        double stoppingDistance =
                (profileVelocity * profileVelocity) /
                        (2.0 * maxAcceleration);

        if (Math.abs(errorToGoal) <= stoppingDistance) {
            desiredAccel = -Math.signum(profileVelocity) * maxAcceleration;
        }

        // Integrate profile
        profileVelocity += desiredAccel * dt;
        profileVelocity =
                clamp(profileVelocity, -maxVelocity, maxVelocity);

        profilePosition += profileVelocity * dt;

        double xRef = profilePosition;
        double vRef = profileVelocity;
        double aRef = (vRef - lastVelocityRef) / dt;
        lastVelocityRef = vRef;

        /* -------- PID (tracking error) -------- */
        double positionError = xRef - currentPosition;
        double velocityError = vRef - currentVelocity;

        double pTerm = kP * positionError;
        double dTerm = kD * velocityError;

        /* -------- Feedforward -------- */
        double ffVolts =
                kV * vRef +
                        kA * aRef +
                        kS * Math.signum(vRef);

        /* -------- Combine -------- */
        double batteryVoltage =
                hardwareMap.voltageSensor.iterator().next().getVoltage();

        double outputVolts = pTerm + dTerm + ffVolts;
        double outputPower = outputVolts / batteryVoltage;

        outputPower = clamp(outputPower, -maxPower, maxPower);
        motor.setPower(outputPower);

        /* -------- Telemetry -------- */
        telemetryM.addLine("Profiled PIDF Position Test");
        telemetryM.addLine("----------------------------");
        telemetryM.addData("Target", targetPosition);
        telemetryM.addData("xRef", xRef);
        telemetryM.addData("Current Pos", currentPosition);
        telemetryM.addData("vRef", vRef);
        telemetryM.addData("Current Vel", currentVelocity);
        telemetryM.addData("Pos Error", positionError);
        telemetryM.addData("Power", outputPower);
        telemetryM.addData("Current (A)",
                motor.getCurrent(CurrentUnit.AMPS));

        telemetryM.update(telemetry);

        lastError = positionError;
    }

    /* ---------------- Utilities ---------------- */

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
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
    Flickers flickers = new Flickers(hardwareMap);
    @Override
    public void init() {
        flickers.init();
    }

    @Override
    public void loop() {
        flickers.setLeftFlickerPos(gamepad1.left_trigger);
        flickers.setRightFlickerPos(gamepad1.right_trigger);
        telemetry.addData("left flicker pos", gamepad1.left_trigger);
        telemetry.addData("right flicker pos", gamepad1.right_trigger);
        telemetry.update();
    }
}


@Configurable
class KeCharacterizationOpMode extends OpMode {
    DcMotorEx motor;

    static int SAMPLES = 5;
    double[] powerLevels = new double[SAMPLES];
    {
        for (int i = 0; i < SAMPLES; i++) {
            powerLevels[i] = i * (1.0 / (SAMPLES));
        }
    }
    int index = 0;
    double targetVelocityThreshold = 25;
    long settleTimeMs = 500;

    double lastVelocity = 0.0;
    long stableStartTime = 0;
    boolean steady = false;
    String TAG = "KeCharacterization";

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void init_loop() {
        telemetry.addLine("Ke Characterization Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        index = 0;
        applyPower();
        lastVelocity = motor.getVelocity();
        stableStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        if (index >= powerLevels.length) {
            motor.setPower(0);
            telemetry.addLine("Done");
            telemetry.addData("LogCat tag", TAG);
            return;
        }

        double currentVelocity = motor.getVelocity();
        double accel = (currentVelocity - lastVelocity) / 0.02; // ~20ms loop

        if (gamepad1.yWasPressed()) {
            logPoint();
            index++;
            applyPower();
        }

        lastVelocity = currentVelocity;

        telemetry.addData("Index", index);
        telemetry.addData("Velocity (ticks/s)", currentVelocity);
        telemetry.addData("Accel (ticks/s^2)", accel);

        telemetry.update();
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

        telemetry.addLine("=== DATA POINT ===");
        telemetry.addData("Power", powerLevels[index]);
        telemetry.addData("Velocity (ticks/s)", velocity);
        telemetry.addData("Battery Voltage (V)", batteryVoltage);
        telemetry.addData("Applied Voltage (V)", appliedVoltage);
        Log.d(TAG, String.format("DATA_POINT,%.3f,%.3f,%.3f,%.3f",
                    powerLevels[index], velocity, batteryVoltage, appliedVoltage)
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

class ShooterOpMode extends OpMode{
    OpMode motor, servo;
    public ShooterOpMode(OpMode motor, OpMode servo) {
        this.motor = motor;
        this.servo = servo;
    }
    @Override
    public void init() {
        motor.hardwareMap = hardwareMap;
        servo.hardwareMap = hardwareMap;
        motor.telemetry = telemetry;
        servo.telemetry = telemetry;
        motor.gamepad1 = gamepad1;
        servo.gamepad1 = gamepad1;
        motor.init();
        servo.init();
    }

    @Override
    public void init_loop() {
        motor.init_loop();
        servo.init_loop();
    }

    @Override
    public void loop() {
        motor.loop();
        servo.loop();
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

        telemetry.addLine();

        if (minVoltage < Double.POSITIVE_INFINITY) {
            telemetry.addData("Robot Battery Voltage (V)", "%.2f", minVoltage);
        } else {
            telemetry.addLine("No valid voltage readings");
        }

        telemetry.update();
    }
}

