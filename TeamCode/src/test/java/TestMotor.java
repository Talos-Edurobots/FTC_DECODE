import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorCoefficientScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFPositionController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFVelocityController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.VelocityUnit;
import org.junit.Test;

public class TestMotor {
    @Test
    public void testController() {
        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_312_RPM);
        MotionState motionState = new MotionState(Angle.fromRadians(1), AngularVelocity.fromRadPerSec(0), 0);
        MotionState target = new MotionState(Angle.fromRadians(0), AngularVelocity.fromRadPerSec(0), 0);
        MotorController cont = new PIDFFPositionController(new PIDFFCoefficients(1, 0, 0, 1, 0, 0));
        LoopState loopState = new LoopState();
        loopState.set((double) 1 /30, 0);
        assertEquals(-2, cont.update(target, motionState, loopState.getDt()), 1e-9);
    }
    @Test
    public void testVelocityUnitConverter() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_6000_RPM;
        EncoderConverter encoder = new EncoderConverter(motorType);
        AngularVelocity vel = AngularVelocity.fromRpm(6000);
        assertEquals(2800, encoder.velocityToTicksPerSecond(vel), 10);
    }
    @Test
    public void testAngleUnitConverter() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_312_RPM;
        EncoderConverter encoder = new EncoderConverter(motorType);
        Angle angle = Angle.fromDegrees(360);
        assertEquals(537.7, encoder.angleToTicks(angle), 0.1);
    }
    @Test
    public void testAngleUnitConverter2() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_312_RPM;
        EncoderConverter encoder = new EncoderConverter(motorType);
        Angle angle = encoder.ticksToAngle(537.7);
        assertEquals(Math.toRadians(360), angle.toRadians(), 1e-3);
    }
    @Test
    public void testVelocityUnitConverter2() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_312_RPM;
        EncoderConverter encoder = new EncoderConverter(motorType);
        AngularVelocity vel = encoder.ticksPerSecondToVelocity(2800);
        assertEquals(312, vel.toRpm(), 1);
    }
    @Test
    public void testVelocityFacadeMathPreservesLegacyTickTuning() {
        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_6000_RPM);
        PIDFFCoefficients legacyCoefficients =
                new PIDFFCoefficients(.005, 0, 0, .02, 0.0052684109772247485, 0);
        PIDFFCoefficients scaledCoefficients =
                MotorCoefficientScaler.fromLegacyTickSpace(legacyCoefficients, encoder);
        PIDFFVelocityController controller = new PIDFFVelocityController(scaledCoefficients);

        double targetTicksPerSecond = 2400.0;
        double currentTicksPerSecond = 1800.0;
        double dt = 0.05;

        MotionState reference = new MotionState(
                Angle.fromRadians(0.0),
                encoder.ticksPerSecondToVelocity(targetTicksPerSecond),
                0.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                encoder.ticksPerSecondToVelocity(currentTicksPerSecond),
                0.0
        );

        double expected =
                legacyCoefficients.kp() * (targetTicksPerSecond - currentTicksPerSecond) +
                        (legacyCoefficients.ks() * Math.signum(targetTicksPerSecond)
                                + legacyCoefficients.kv() * targetTicksPerSecond) / 12.0;
        double actual = controller.update(reference, current, dt, 1.0 / 12.0);

        assertEquals(expected, actual, 1e-9);
    }
    @Test
    public void testLoopStateBatteryHelper() {
        LoopState loopState = new LoopState();
        loopState.setBatteryVoltage(10.0);

        assertEquals(0.1, loopState.getBatteryVoltageFactor(), 1e-9);
        assertEquals(10.0, loopState.getBatteryVoltage(), 1e-9);
    }
}
