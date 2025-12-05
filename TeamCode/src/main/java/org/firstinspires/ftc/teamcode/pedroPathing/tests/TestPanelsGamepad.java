package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Panels Gamepad", group = "tests")
public class TestPanelsGamepad extends LinearOpMode {
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    GamepadManager gamepad = PanelsGamepad.INSTANCE.getFirstManager();
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad.asCombinedFTCGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            panelsTelemetry.addData("gamepad dpad left from panels", gamepad.getDpadLeft());
            panelsTelemetry.addData("gamepad dpad right from driver station", gamepad1.dpad_right);
            panelsTelemetry.update();
        }
    }
}
