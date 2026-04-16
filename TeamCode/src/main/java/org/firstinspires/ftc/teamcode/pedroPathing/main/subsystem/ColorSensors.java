package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensors {
    HardwareMap hwmap;
    DistanceSensor color1, color2, color3;
    double dist1, dist2, dist3;
    boolean wasFull;
    ElapsedTime timer = new ElapsedTime();
    public ColorSensors() {}
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        color1 = hwmap.get(DistanceSensor.class, "color1");
        color2 = hwmap.get(DistanceSensor.class, "color2");
        color3 = hwmap.get(DistanceSensor.class, "color3");
    }
    public void update() {
        dist1 = color1.getDistance(DistanceUnit.CM);
        dist2 = color2.getDistance(DistanceUnit.CM);
        dist3 = color3.getDistance(DistanceUnit.CM);
        if (!wasFull)  timer.reset();
        wasFull = is1Detected() && is2Detected() && is3Detected();
    }
    public boolean is1Detected() {
        return dist1 < 6;
    }
    public boolean is2Detected() {
        return dist2 < 3;
    }
    public boolean is3Detected() {
        return dist3 < 7;
    }
    public boolean isFull() {
        return timer.seconds() > 0.2;
    }
    public double getFullTIme() {
        return timer.seconds();
    }
    public double getColor1() {
        return dist1;
    }
    public double getColor2() {
        return dist2;
    }
    public double getColor3() {
        return dist3;
    }
}
