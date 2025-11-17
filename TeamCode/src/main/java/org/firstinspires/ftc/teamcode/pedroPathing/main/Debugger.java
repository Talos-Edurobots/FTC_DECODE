package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.Consumer;
import java.util.function.Supplier;

@TeleOp(name = "Debugger", group = "main")
public class Debugger extends SelectableOpMode {
    public Debugger() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Without encoder", l -> {
                l.add("Run Intake", IntakePowerTest::new);
            });

        });
    };
    @Override
    public void onSelect(){}
}

class IntakePowerTest extends OpMode {
    DcMotorEx intake;
    double maxVelocity = 0;
    @Override
    public void init_loop() {
        double maxVelocity = 0;
        DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get(RobotConstants.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        double currentVelocity = intake.getVelocity();
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Intake Velocity", currentVelocity);
        telemetry.addData("max Velocity", maxVelocity);
        maxVelocity = Math.max(maxVelocity, currentVelocity);
        telemetry.update();
    }
}


