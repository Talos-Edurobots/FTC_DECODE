package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;

public final class MotorCoefficientScaler {
    private MotorCoefficientScaler() {}

    public static PIDFFCoefficients fromLegacyTickSpace(PIDFFCoefficients legacyCoefficients,
                                                        EncoderConverter encoderConverter) {
        double ticksPerRadian = encoderConverter.angleToTicks(Angle.fromRadians(1.0));
        return new PIDFFCoefficients(
                legacyCoefficients.kp() * ticksPerRadian,
                legacyCoefficients.ki() * ticksPerRadian,
                legacyCoefficients.kd() * ticksPerRadian,
                legacyCoefficients.ks(),
                legacyCoefficients.kv() * ticksPerRadian,
                legacyCoefficients.ka() * ticksPerRadian
        );
    }

    public static MotionProfilingCoefficients fromLegacyTickSpace(
            MotionProfilingCoefficients legacyCoefficients,
            EncoderConverter encoderConverter
    ) {
        return new MotionProfilingCoefficients(
                fromLegacyTickSpace(legacyCoefficients.getPidCoef(), encoderConverter),
                encoderConverter.ticksPerSecondToVelocity(legacyCoefficients.getMaxVelocity()).toRadPerSec(),
                encoderConverter.ticksPerSecondSquaredToAcceleration(legacyCoefficients.getMaxAcceleration()),
                encoderConverter.ticksPerSecondSquaredToAcceleration(legacyCoefficients.getMaxDeceleration())
        );
    }
}
