package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;

public class  Hang {
    public MotorConfig motor = RobotConstants.HANG_CONFIG;
    HardwareMap hwmap;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        motor.init(hwmap);
    }
    public void update(double power, int posInDegrees) {
        motor.setPositionInTicks(posInDegrees * motor.getMotorType().getTicksPerDegree());
        motor.updateSimplePositionControl();
        telemetryM.addData("hang current", motor.getCurrent());
        telemetryM.addData("hang current", motor.getCurrentPosition());
        telemetryM.addData("hang target", motor.getTargetPositionTicks());
        telemetryM.addData("hang power", motor.getPower());
    }
}
