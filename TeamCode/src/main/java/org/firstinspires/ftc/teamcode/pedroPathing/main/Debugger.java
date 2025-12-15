package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {
    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", ne -> {
                ne.add("Run Intake", () -> new MotorPowerTest(RobotConstants.INTAKE_NAME));
                ne.add("Run Shooter", () -> new MotorPowerTest(RobotConstants.SHOOTER_NAME));
                ne.add("Run Left Front Drive", () -> new MotorPowerTest(RobotConstants.LEFT_FRONT_NAME));
                ne.add("Run Right Front Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_FRONT_NAME));
                ne.add("Run Left Back Drive", () -> new MotorPowerTest(RobotConstants.LEFT_BACK_NAME));
                ne.add("Run Right Back Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_BACK_NAME));
            });
            s.folder("Velocity Control", vc -> {;
                vc.add("Run Shooter Velocity", () -> new MotorVelocityTest(RobotConstants.SHOOTER_NAME));
                vc.add("Run Intake Velocity", () -> new MotorVelocityTest(RobotConstants.INTAKE_NAME));
            });
            s.folder("servo control", sc -> {
                sc.add("right servo", () -> new ServoControl(RobotConstants.RIGHT_SERVO_NAME));
                sc.add("left servo", () -> new ServoControl(RobotConstants.LEFT_SERVO_NAME));
                sc.add("right flicker", () -> new ServoControl(RobotConstants.RIGHT_FLICKER_NAME));
                sc.add("left flicker", () -> new ServoControl(RobotConstants.LEFT_FLICKER_NAME));
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
    public MotorPowerTest(String motorName) {
        this.motorName = motorName;
    }
    private final String motorName;
    private DcMotorEx motor;

    private double maxVelocity = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            motor.setDirection(motor.getDirection() == DcMotor.Direction.FORWARD ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        // Set power via triggers
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        motor.setPower(power);

        double currentVelocity = motor.getVelocity();
        maxVelocity = Math.max(maxVelocity, currentVelocity);

        telemetry.addData("Intake Power", power);
        telemetry.addData("Intake Velocity", currentVelocity);
        telemetry.addData("current Direction", motor.getDirection().toString());
        telemetry.addData("current", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.update();
    }
}

@Configurable
class MotorVelocityTest extends OpMode {
    public double maxVelocity = 2700;
    public MotorVelocityTest(String motorName) {
        this.motorName = motorName;
    }
    private final String motorName;
    private DcMotorEx motor;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double targetVelocity = 0, k_p = 0, k_i = 0, k_d = 0, k_s = 0, k_u = 0;
    double errorSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);

        telemetryM.addLine("Init Complete");
        telemetryM.update(telemetry);
        timer.startTime();
    }

    @Override
    public void loop() {

        timer.reset();
        // Set velocity via triggers
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;
        double derivative = (error - lastError) / timer.seconds();
        errorSum += error;
        motor.setPower(
                k_p * error +
                k_i * errorSum + (error * timer.seconds()) +
                k_d * derivative+
                k_s /** Math.signum(targetVelocity)*/ +
                k_u * targetVelocity
        );
        telemetryM.addData("s", k_s);
        telemetryM.addData("Current Velocity", currentVelocity);
        telemetryM.addData("power", motor.getPower());
        telemetryM.addData("current", motor.getCurrent(CurrentUnit.AMPS));
//        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM.update(telemetry);
        lastError = error;
    }
}

    class ServoControl extends OpMode{
        String servoName;
        Servo servo;
        public ServoControl(String servoName) {
            this.servoName = servoName;
        }

        @Override
        public void init() {
            servo = hardwareMap.servo.get(servoName);
            telemetry.addLine("init");
            telemetry.update();
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
            servo.setPosition(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("servo direction", servo.getDirection());
            telemetry.addData("servo pos", servo.getPosition());
            telemetry.update();
        }
    }
