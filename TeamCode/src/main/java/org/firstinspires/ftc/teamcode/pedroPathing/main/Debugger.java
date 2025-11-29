package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.update();
    }
}

class MotorVelocityTest extends OpMode {
    public MotorVelocityTest(String motorName) {
        this.motorName = motorName;
    }
    private final String motorName;
    private DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Set velocity via triggers
        double targetVelocity = 2400 * (gamepad1.right_trigger - gamepad1.left_trigger);
        motor.setVelocity(targetVelocity);

        double currentVelocity = motor.getVelocity();

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();
    }
}