package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.drivetrain.DriveTrain;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    DriveTrain driveTrain;
    IMU imu;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;
        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);
        imu.resetYaw();
        waitForStart();
        while (opModeIsActive()){
            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.aWasPressed()) {
                intake.setPower(1);
            }
            else if (gamepad1.bWasPressed()) {
                intake.setPower(0);
            }
            else if (gamepad1.yWasPressed()) {
                intake.setPower(-1);
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double speed = 1;
            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);

            telemetryM.addData("Heading", heading);
            telemetryM.addData("dx", dt);
            telemetryM.addData("intake velocity", intake.getVelocity());
            telemetryM.addData("intake current", intake.getCurrent(CurrentUnit.AMPS));
            telemetryM.update(telemetry);

            sleep(20);

        }
    }
}
