package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;

public class EncoderConverter {
    private final GoBILDAMotorTypes motorType;

    public EncoderConverter(GoBILDAMotorTypes motorType) {
        this.motorType = motorType;
    }

    public double angleToTicks(Angle angle) {
        return angle.toRadians() * motorType.getTicksPerOutputRev() / (2 * Math.PI);
    }

    public Angle ticksToAngle(double ticks) {
        return new Angle(ticks * (2 * Math.PI) / motorType.getTicksPerOutputRev());
    }
    public double velocityToTicksPerSecond(AngularVelocity vel) {
        return vel.toRadPerSec() * motorType.getTicksPerOutputRev() / (2 * Math.PI);
    }

    public AngularVelocity ticksPerSecondToVelocity(double ticksPerSec) {
        return new AngularVelocity(ticksPerSec * (2 * Math.PI) / motorType.getTicksPerOutputRev());
    }

    public double accelerationToTicksPerSecondSquared(double accelerationRadPerSecSquared) {
        return accelerationRadPerSecSquared * motorType.getTicksPerOutputRev() / (2 * Math.PI);
    }

    public double ticksPerSecondSquaredToAcceleration(double ticksPerSecondSquared) {
        return ticksPerSecondSquared * (2 * Math.PI) / motorType.getTicksPerOutputRev();
    }
}
