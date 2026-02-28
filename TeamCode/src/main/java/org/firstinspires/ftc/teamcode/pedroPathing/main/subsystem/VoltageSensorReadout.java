package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageSensorReadout {
    private VoltageSensor voltageSensor;
    String deviceName = "Control Hub";
    public void init(HardwareMap hwMap) {
        voltageSensor = hwMap.voltageSensor.get(deviceName);
    }
    public double getVoltage() {
        double voltage = voltageSensor.getVoltage();
        return voltage;
    }
}
