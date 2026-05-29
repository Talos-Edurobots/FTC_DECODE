import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorCoefficientScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.OpenLoopMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.ProfiledPositionMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.VelocityControlledMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFPositionController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFVelocityController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.TrapezoidalMotionProfileController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;
import org.junit.Test;

public class MotorPackageTest {
    @Test
    public void angleSupportsConversionAndArithmetic() {
        Angle angle = Angle.fromDegrees(90);
        Angle sum = angle.add(Angle.fromRadians(Math.PI / 2.0));
        Angle difference = sum.subtract(Angle.fromDegrees(45));

        assertEquals(Math.PI / 2.0, angle.toRadians(), 1e-9);
        assertEquals(180.0, sum.toDegrees(), 1e-9);
        assertEquals(135.0, difference.toDegrees(), 1e-9);
    }

    @Test
    public void angularVelocitySupportsConversionAndArithmetic() {
        AngularVelocity velocity = AngularVelocity.fromRpm(60);
        AngularVelocity sum = velocity.add(AngularVelocity.fromRadPerSec(Math.PI));
        AngularVelocity difference = sum.subtract(AngularVelocity.fromRadPerSec(Math.PI));

        assertEquals(2.0 * Math.PI, velocity.toRadPerSec(), 1e-9);
        assertEquals(90.0, sum.toRpm(), 1e-9);
        assertEquals(60.0, difference.toRpm(), 1e-9);
    }

    @Test
    public void encoderConverterConvertsAngleVelocityAndAcceleration() {
        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_312_RPM);
        assertEquals(537.7, encoder.angleToTicks(Angle.fromDegrees(360)), 0.1);
        assertEquals(Math.toRadians(360), encoder.ticksToAngle(537.7).toRadians(), 1e-3);
        assertEquals(312.0, encoder.ticksPerSecondToVelocity(2800).toRpm(), 1.0);
        assertEquals(2800.0, encoder.velocityToTicksPerSecond(AngularVelocity.fromRpm(312)), 10.0);
        assertEquals(537.7, encoder.accelerationToTicksPerSecondSquared(2.0 * Math.PI), 0.1);
        assertEquals(2.0 * Math.PI, encoder.ticksPerSecondSquaredToAcceleration(537.7), 1e-3);
    }

    @Test
    public void gobildaMotorTypeExposesExpectedDerivedValues() {
        GoBILDAMotorTypes motorType = GoBILDAMotorTypes.MOTOR_312_RPM;

        assertEquals(312.0, motorType.getRpm(), 1e-9);
        assertEquals(
                GoBILDAMotorTypes.ENCODER_TICKS_PER_MOTOR_REV * motorType.getGearRatio(),
                motorType.getTicksPerOutputRev(),
                1e-9
        );
        assertEquals(motorType.getTicksPerOutputRev() / 360.0, motorType.getTicksPerDegree(), 1e-9);
        assertEquals(
                motorType.getTicksPerOutputRev() / (2.0 * Math.PI),
                motorType.getTicksPerRadian(),
                1e-9
        );
    }

    @Test
    public void loopStateStoresDtAndBatteryHelpers() {
        LoopState loopState = new LoopState();
        loopState.set(0.02, 1.0 / 12.5);

        assertEquals(0.02, loopState.getDt(), 1e-9);
        assertEquals(1.0 / 12.5, loopState.getBatteryVoltageFactor(), 1e-9);
        assertEquals(12.5, loopState.getBatteryVoltage(), 1e-9);

        loopState.setBatteryVoltage(10.0);
        assertEquals(0.1, loopState.getBatteryVoltageFactor(), 1e-9);
        assertEquals(10.0, loopState.getBatteryVoltage(), 1e-9);
    }

    @Test
    public void motionStateExposesProvidedValues() {
        MotionState state = new MotionState(
                Angle.fromRadians(1.2),
                AngularVelocity.fromRadPerSec(3.4),
                5.6
        );

        assertEquals(1.2, state.getPosition().toRadians(), 1e-9);
        assertEquals(3.4, state.getVelocity().toRadPerSec(), 1e-9);
        assertEquals(5.6, state.getAcceleration(), 1e-9);
    }

    @Test
    public void motorLimitsDefaultsAreSafe() {
        MotorLimits defaults = MotorLimits.defaults();

        assertEquals(1.0, defaults.getMaxPower(), 1e-9);
        assertTrue(Double.isInfinite(defaults.getCurrentAlertAmps()));
    }

    @Test
    public void positionControllerComputesPidAndFeedforward() {
        PIDFFPositionController controller =
                new PIDFFPositionController(new PIDFFCoefficients(2.0, 0.5, 3.0, 0.2, 0.7, 0.9));
        MotionState reference = new MotionState(
                Angle.fromRadians(2.0),
                AngularVelocity.fromRadPerSec(5.0),
                7.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(1.0),
                AngularVelocity.fromRadPerSec(3.0),
                0.0
        );

        double output = controller.update(reference, current, 0.5, 0.25, 1.0);

        double expectedPid = 2.0 * 1.0 + 0.5 * 0.5 + 3.0 * 2.0;
        double expectedFf = (0.2 + 0.7 * 5.0 + 0.9 * 7.0) * 0.25;
        assertEquals(expectedPid + expectedFf, output, 1e-9);
    }

    @Test
    public void positionControllerResetClearsIntegralState() {
        PIDFFPositionController controller =
                new PIDFFPositionController(new PIDFFCoefficients(0.0, 2.0, 0.0, 0.0, 0.0, 0.0));
        MotionState reference = new MotionState(
                Angle.fromRadians(1.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );

        double first = controller.update(reference, current, 0.5);
        double second = controller.update(reference, current, 0.5);
        controller.reset();
        double afterReset = controller.update(reference, current, 0.5);

        assertTrue(second > first);
        assertEquals(first, afterReset, 1e-9);
    }

    @Test
    public void velocityControllerComputesPidDerivativeAndFeedforward() {
        PIDFFVelocityController controller =
                new PIDFFVelocityController(new PIDFFCoefficients(2.0, 0.5, 4.0, 0.3, 0.7, 1.1));
        MotionState reference = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(5.0),
                2.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(3.0),
                0.0
        );

        double first = controller.update(reference, current, 0.5, 0.25);
        double second = controller.update(reference, current, 0.5, 0.25);

        double expectedFirstPid = 2.0 * 2.0 + 0.5 * 1.0;
        double expectedFf = (0.3 + 0.7 * 5.0 + 1.1 * 2.0) * 0.25;
        assertEquals(expectedFirstPid + expectedFf, first, 1e-9);
        assertEquals(first + 0.5, second, 1e-9);
    }

    @Test
    public void velocityControllerResetClearsIntegralAndDerivativeState() {
        PIDFFVelocityController controller =
                new PIDFFVelocityController(new PIDFFCoefficients(0.0, 2.0, 5.0, 0.0, 0.0, 0.0));
        MotionState reference = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(1.0),
                0.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );

        double first = controller.update(reference, current, 0.5);
        double second = controller.update(reference, current, 0.5);
        controller.reset();
        double afterReset = controller.update(reference, current, 0.5);

        assertTrue(second > first);
        assertEquals(first, afterReset, 1e-9);
    }

    @Test
    public void trapezoidalProfileControllerAdvancesTowardTarget() {
        PIDFFCoefficients pidf = new PIDFFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        TrapezoidalMotionProfileController controller =
                new TrapezoidalMotionProfileController(
                        new MotionProfilingCoefficients(pidf, 4.0, 2.0)
                );
        MotionState target = new MotionState(
                Angle.fromRadians(10.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );

        controller.update(target, current, 0.5);
        MotionState refState = controller.getReferenceState();

        assertEquals(0.5, refState.getPosition().toRadians(), 1e-9);
        assertEquals(1.0, refState.getVelocity().toRadPerSec(), 1e-9);
        assertEquals(2.0, refState.getAcceleration(), 1e-9);
    }

    @Test
    public void trapezoidalProfileResetUsesMeasuredState() {
        TrapezoidalMotionProfileController controller =
                new TrapezoidalMotionProfileController(
                        new MotionProfilingCoefficients(
                                new PIDFFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                4.0,
                                2.0
                        )
                );
        MotionState measured = new MotionState(
                Angle.fromRadians(3.0),
                AngularVelocity.fromRadPerSec(1.5),
                -0.5
        );

        controller.reset(measured);
        MotionState reference = controller.getReferenceState();

        assertEquals(3.0, reference.getPosition().toRadians(), 1e-9);
        assertEquals(1.5, reference.getVelocity().toRadPerSec(), 1e-9);
        assertEquals(-0.5, reference.getAcceleration(), 1e-9);
    }

    @Test
    public void motorCoefficientScalerConvertsLegacyTickCoefficients() {
        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_6000_RPM);
        PIDFFCoefficients legacy = new PIDFFCoefficients(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        PIDFFCoefficients scaled = MotorCoefficientScaler.fromLegacyTickSpace(legacy, encoder);
        double ticksPerRadian = encoder.angleToTicks(Angle.fromRadians(1.0));

        assertEquals(1.0 * ticksPerRadian, scaled.kp(), 1e-9);
        assertEquals(2.0 * ticksPerRadian, scaled.ki(), 1e-9);
        assertEquals(3.0 * ticksPerRadian, scaled.kd(), 1e-9);
        assertEquals(4.0, scaled.ks(), 1e-9);
        assertEquals(5.0 * ticksPerRadian, scaled.kv(), 1e-9);
        assertEquals(6.0 * ticksPerRadian, scaled.ka(), 1e-9);
    }

    @Test
    public void motionProfilingScalerConvertsVelocityAndAccelerationLimits() {
        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_312_RPM);
        MotionProfilingCoefficients legacy = new MotionProfilingCoefficients(
                new PIDFFCoefficients(1.0, 0.0, 0.0, 2.0, 3.0, 4.0),
                1200.0,
                2400.0
        );

        MotionProfilingCoefficients scaled =
                MotorCoefficientScaler.fromLegacyTickSpace(legacy, encoder);

        assertEquals(
                encoder.ticksPerSecondToVelocity(1200.0).toRadPerSec(),
                scaled.getMaxVelocity(),
                1e-9
        );
        assertEquals(
                encoder.ticksPerSecondSquaredToAcceleration(2400.0),
                scaled.getMaxAcceleration(),
                1e-9
        );
        assertEquals(2.0, scaled.getPidCoef().ks(), 1e-9);
    }

    @Test
    public void metaMotorRejectsAccessBeforeInitialization() {
        MetaMotor motor = new MetaMotor();

        assertThrows(IllegalStateException.class, motor::getPower);
        assertThrows(IllegalStateException.class, () -> motor.setPower(0.1));
        assertThrows(IllegalStateException.class, motor::getCurrentPositionTicks);
        assertThrows(IllegalStateException.class, motor::getVelocityTicksPerSecond);
        assertThrows(IllegalStateException.class, motor::getCurrentAmps);
        assertThrows(IllegalStateException.class, motor::isOverCurrent);
        assertThrows(IllegalStateException.class, motor::getMode);
    }

    @Test
    public void openLoopMotorDelegatesPowerToHardware() {
        FakeMetaMotor hardware = new FakeMetaMotor();
        OpenLoopMotor openLoopMotor = new OpenLoopMotor(hardware);

        openLoopMotor.setPower(0.4);

        assertEquals(0.4, openLoopMotor.getPower(), 1e-9);
        assertEquals(0.4, hardware.power, 1e-9);
    }

    @Test
    public void velocityControlledMotorTracksTargetAndWritesPower() {
        FakeMetaMotor hardware = new FakeMetaMotor();
        hardware.velocityTicksPerSecond = 1800.0;
        PIDFFCoefficients coefficients = new PIDFFCoefficients(1.0, 0.0, 0.0, 0.2, 0.5, 0.0);
        VelocityControlledMotor motor =
                new VelocityControlledMotor(hardware, GoBILDAMotorTypes.MOTOR_6000_RPM, coefficients);
        LoopState loopState = new LoopState();
        loopState.setBatteryVoltage(12.0);
        loopState.set(0.1, loopState.getBatteryVoltageFactor());

        motor.setTargetVelocityTicksPerSecond(2400.0);
        double power = motor.update(loopState);

        EncoderConverter encoder = new EncoderConverter(GoBILDAMotorTypes.MOTOR_6000_RPM);
        double targetRadPerSec = encoder.ticksPerSecondToVelocity(2400.0).toRadPerSec();
        double currentRadPerSec = encoder.ticksPerSecondToVelocity(1800.0).toRadPerSec();
        double expected =
                (targetRadPerSec - currentRadPerSec) +
                        (0.2 + 0.5 * targetRadPerSec) / 12.0;

        assertEquals(2400.0, motor.getTargetVelocityTicksPerSecond(), 1e-9);
        assertEquals(1800.0, motor.getMeasuredVelocityTicksPerSecond(), 1e-9);
        assertEquals(expected, power, 1e-9);
        assertEquals(expected, hardware.power, 1e-9);
    }

    @Test
    public void profiledPositionMotorClipsTargetToConfiguredLimits() {
        FakeMetaMotor hardware = new FakeMetaMotor();
        ProfiledPositionMotor motor = new ProfiledPositionMotor(
                hardware,
                GoBILDAMotorTypes.MOTOR_312_RPM,
                new MotionProfilingCoefficients(
                        new PIDFFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        4.0,
                        2.0
                ),
                1.0
        );

        motor.setAngleLimits(Angle.fromRadians(-1.0), Angle.fromRadians(1.0));
        motor.setTargetAngle(Angle.fromRadians(3.0));

        assertEquals(1.0, motor.getTargetAngle().toRadians(), 1e-9);
    }

    @Test
    public void profiledPositionMotorUpdateProducesPowerAndReferenceState() {
        FakeMetaMotor hardware = new FakeMetaMotor();
        hardware.currentPositionTicks = 0;
        hardware.velocityTicksPerSecond = 0.0;
        ProfiledPositionMotor motor = new ProfiledPositionMotor(
                hardware,
                GoBILDAMotorTypes.MOTOR_312_RPM,
                new MotionProfilingCoefficients(
                        new PIDFFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        4.0,
                        2.0
                ),
                1.0
        );
        LoopState loopState = new LoopState();
        loopState.setBatteryVoltage(12.0);
        loopState.set(0.5, loopState.getBatteryVoltageFactor());

        motor.setTargetAngle(Angle.fromRadians(10.0));
        double power = motor.update(loopState);
        MotionState reference = motor.getReferenceState();

        assertEquals(0.0, power, 1e-9);
        assertEquals(0.5, reference.getPosition().toRadians(), 1e-9);
        assertEquals(1.0, reference.getVelocity().toRadPerSec(), 1e-9);
        assertEquals(2.0, reference.getAcceleration(), 1e-9);
    }

    @Test
    public void profiledPositionMotorSetPowerResetsProfileState() {
        FakeMetaMotor hardware = new FakeMetaMotor();
        ProfiledPositionMotor motor = new ProfiledPositionMotor(
                hardware,
                GoBILDAMotorTypes.MOTOR_312_RPM,
                new MotionProfilingCoefficients(
                        new PIDFFCoefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        4.0,
                        2.0
                ),
                1.0
        );
        LoopState loopState = new LoopState();
        loopState.setBatteryVoltage(12.0);
        loopState.set(0.5, loopState.getBatteryVoltageFactor());

        motor.setTargetAngle(Angle.fromRadians(10.0));
        motor.update(loopState);
        assertNotEquals(0.0, motor.getReferenceState().getPosition().toRadians(), 0.0);

        motor.setPower(0.25);

        assertEquals(0.25, hardware.power, 1e-9);
        assertEquals(0.0, motor.getReferenceState().getPosition().toRadians(), 1e-9);
        assertEquals(0.0, motor.getReferenceState().getVelocity().toRadPerSec(), 1e-9);
    }

    private static final class FakeMetaMotor extends MetaMotor {
        private double power;
        private int currentPositionTicks;
        private double velocityTicksPerSecond;
        private double currentAmps;
        private boolean overCurrent;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private int targetPosition;

        @Override
        public void setPower(double power) {
            this.power = power;
        }

        @Override
        public double getPower() {
            return power;
        }

        @Override
        public void setMode(DcMotor.RunMode runMode) {
            this.mode = runMode;
        }

        @Override
        public DcMotor.RunMode getMode() {
            return mode;
        }

        @Override
        public int getCurrentPositionTicks() {
            return currentPositionTicks;
        }

        @Override
        public double getVelocityTicksPerSecond() {
            return velocityTicksPerSecond;
        }

        @Override
        public double getCurrentAmps() {
            return currentAmps;
        }

        @Override
        public boolean isOverCurrent() {
            return overCurrent;
        }

        @Override
        public void setTargetPosition(int targetPositionTicks) {
            this.targetPosition = targetPositionTicks;
        }

        @Override
        public int getTargetPosition() {
            return targetPosition;
        }
    }
}
