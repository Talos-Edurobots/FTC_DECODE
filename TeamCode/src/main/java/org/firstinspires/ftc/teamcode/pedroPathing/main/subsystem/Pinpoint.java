package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pinpoint {
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    private GoBildaPinpointDriver pinpoint;
    private HardwareMap hardwareMap;
    public Pinpoint(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT_NAME);
        pinpoint.setEncoderDirections(RobotConstants.PINPOINT_FORWARD_DIRECTION, RobotConstants.PINPOINT_STRAFE_DIRECTION);
        pinpoint.setOffsets(RobotConstants.PINPOINT_STRAFE_POD_X, RobotConstants.PINPOINT_FORWARD_POD_Y, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        pinpoint.initialize();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose getPosition() {
        return new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );
    }
}
