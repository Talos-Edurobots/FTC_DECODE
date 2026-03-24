import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFPositionController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Velocity;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.VelocityUnit;
import org.junit.Test;

public class TestMotor {

    @Test
    public void testController() {
        MotionState motionState = new MotionState();
        LoopState loopState = new LoopState();
        MotorController cont = new PIDFFPositionController(new PIDFFCoefficients(0, 0, 0, 1));
        motionState.set(20, 0, 0);
        loopState.set((double) 1 /30, 1);
        assertEquals(-1, cont.update(10, motionState, loopState), 1e-9);
    }
    @Test
    public void testVelocityUnitConverter() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_6000_RPM;
        Velocity vel = new Velocity(6000, VelocityUnit.RPM, motorType);
        assertEquals(2800, vel.get(VelocityUnit.TICKS_PER_SECOND), 10);
    }
}
