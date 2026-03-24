import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.MotorController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFPositionController;


public class TestMotor {
    MotionState motionState = new MotionState();
    LoopState loopState = new LoopState();
    @org.junit.Test
    public void testUnits() {
        MotorController cont = new PIDFFPositionController(new PIDFFCoefficients(0, 0, 0, 1));
        motionState.set(0, 0, 0);
        loopState.set((double) 1 /30, 1);
        assertEquals(1, cont.update(10, motionState, loopState), 1e-9);
    }
}
