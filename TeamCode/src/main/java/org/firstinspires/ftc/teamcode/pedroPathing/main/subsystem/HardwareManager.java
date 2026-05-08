package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

public final class HardwareManager {

    private final List<LynxModule> hubs;

    public HardwareManager(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

    }

    public double getTotalCurrentDrawAmps() {
        double totalCurrent = 0;
        for (LynxModule hub : hubs) {
            totalCurrent += hub.getCurrent(CurrentUnit.AMPS);
        }
        return totalCurrent;
    }
}

