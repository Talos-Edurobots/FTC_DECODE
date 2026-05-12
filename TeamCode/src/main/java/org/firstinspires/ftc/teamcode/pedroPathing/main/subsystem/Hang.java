package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;

public class  Hang {
    private static final double RUN_TO_POSITION_VELOCITY = 2500.0;

    public final MetaMotor motor = new MetaMotor();
    HardwareMap hwmap;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private double targetPositionTicks = 0.0;

    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        motor.hwName(RobotConstants.HANG_MOTOR_NAME);
        motor.direction(RobotConstants.HANG_MOTOR_DIRECTION);
        motor.zeroPowerBehavior(RobotConstants.HANG_ZERO_POWER_BEHAVIOR);
        motor.init(hwmap);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void update(double power, int posInDegrees) {
        targetPositionTicks = posInDegrees * RobotConstants.HANG_MOTOR_TYPE.getTicksPerDegree();
        motor.setTargetPosition((int) targetPositionTicks);
        motor.setVelocity(RUN_TO_POSITION_VELOCITY);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetryM.addData("hang current", motor.getCurrentAmps());
        telemetryM.addData("hang current", motor.getCurrentPositionTicks());
        telemetryM.addData("hang target", targetPositionTicks);
        telemetryM.addData("hang power", motor.getPower());
    }
}
