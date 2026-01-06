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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorUse;
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
            });
            s.folder("ke characterization", kect -> {
                kect.add("shooter ke", () -> new KeCharacterizationOpMode(RobotConstants.SHOOTER_CONFIG));
                kect.add("intake ke", () -> new KeCharacterizationOpMode(RobotConstants.INTAKE_CONFIG));
                kect.add("turret ke", () -> new KeCharacterizationOpMode(RobotConstants.TURRET_CONFIG));
            });
            s.folder("high level", hlt -> {
                hlt.add("flicker analog control", FlickerAnalogControl::new);
                hlt.add("voltage sensor readout", VoltageSensorReadoutOpMode::new);
//                hlt.add("turret position pid", () -> new MotorPositionTest(RobotConstants.TURRET_CONFIG));
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
        kp = motor.kP; ki = motor.kI; kd = motor.kD; ks = motor.kS; kv = motor.kU; ka = motor.kA;
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
        motor.kP = kp; motor.kI = ki; motor.kD = kd; motor.kS = ks; motor.kU = kv; motor.kA = ka;
        motor.setPositionInTicks(targetPosition);
        motor.updateVelocityPIDF(timer.seconds(), 12);
        timer.reset();
        telemetryM.addData("current pos", motor.getCurrentPosition());
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("kp", kp);
        telemetryM.addData("target pos", targetPosition);
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
        timer.reset();
        motor.setVelocityTicksPerSecond(targetVelocity);
        motor.setPIDFCoefficients(kp, ki, kd, ks, kv, ka);
        // Set velocity via triggers
        if (gamepad1.bWasPressed()) {
            runMotor ^= true;
        }
        if (runMotor) {
            motor.updateVelocityPIDF(timer.seconds(), 12);
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
    public KeCharacterizationOpMode(MotorConfig motor) {
        this.motor = motor;
    }
    MotorConfig motor;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    static int SAMPLES = 5;
    double[] powerLevels = new double[SAMPLES];
    {
        for (int i = 1; i <= SAMPLES; i++) {
            powerLevels[i] = i * (1.0 / (SAMPLES));
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
        Log.d(TAG, ",Velocity,applied-voltage");
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
        Log.d(TAG, String.format(",%.3f,%.3f",
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
