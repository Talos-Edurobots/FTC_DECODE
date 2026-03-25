import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFPositionController;
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
        MotionState motionState = new MotionState(encoder.ticksToAngle(10), AngularVelocity.fromRadPerSec(0), 0);
        MotionState target = new MotionState(encoder.ticksToAngle(0), AngularVelocity.fromRadPerSec(0), 0);
        MotorController cont = new PIDFFPositionController(new PIDFFCoefficients(0, 0, 0, 1, 0, 0));
        LoopState loopState = new LoopState();
        loopState.set((double) 1 /30, 0);
        assertEquals(-1, cont.update(target, motionState, loopState.getDt()), 1e-9);
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
        assertEquals(Math.toRadians(360), angle.toRadians(), 0.1);
    }
}
