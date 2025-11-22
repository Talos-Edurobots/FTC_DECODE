package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {

    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", l -> {
                l.add("Run Intake", () -> new MotorPowerTest(RobotConstants.INTAKE_NAME));
                l.add("Run Shooter", () -> new MotorPowerTest(RobotConstants.SHOOTER_NAME));
                l.add("Run Left Front Drive", () -> new MotorPowerTest(RobotConstants.LEFT_FRONT_NAME));
                l.add("Run Right Front Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_FRONT_NAME));
                l.add("Run Left Back Drive", () -> new MotorPowerTest(RobotConstants.LEFT_BACK_NAME));
                l.add("Run Right Back Drive", () -> new MotorPowerTest(RobotConstants.RIGHT_BACK_NAME));
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
    private DcMotorEx intake;

    private double maxVelocity = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, motorName);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Set power via triggers
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        intake.setPower(power);

        double currentVelocity = intake.getVelocity();
        maxVelocity = Math.max(maxVelocity, currentVelocity);

        telemetry.addData("Intake Power", power);
        telemetry.addData("Intake Velocity", currentVelocity);
        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.update();
    }
}
