package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorCoefficientScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.TrapezoidalMotionProfileController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;

/**
 * Facade for motor-shaft profiled position control.
 *
 * <p>This type composes a hardware adapter, encoder conversion, and a
 * trapezoidal-profile controller. Mechanism-space conversion should happen
 * outside this class before a target angle is set.
 */
public class ProfiledPositionMotor {
    private final MetaMotor hardware;
    private final EncoderConverter encoderConverter;
    private final TrapezoidalMotionProfileController controller;
    private double maxPower;

    private Angle targetAngle = Angle.fromRadians(0.0);
    private Angle minAngle = Angle.fromRadians(Double.NEGATIVE_INFINITY);
    private Angle maxAngle = Angle.fromRadians(Double.POSITIVE_INFINITY);
    private Angle targetTolerance = Angle.fromRadians(0.0);

    private double lastMeasuredVelocityRadPerSec = 0.0;
    private boolean hasLastMeasurement = false;
    private boolean profileInitialized = false;

    public ProfiledPositionMotor(String hardwareName,
                                 GoBILDAMotorTypes motorType,
                                 DcMotorSimple.Direction direction,
                                 DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                                 MotionProfilingCoefficients coefficients,
                                 MotorLimits limits) {
        this(new MetaMotor(), motorType, coefficients, limits.getMaxPower());
        hardware.hwName(hardwareName);
        hardware.direction(direction);
        hardware.zeroPowerBehavior(zeroPowerBehavior);
        hardware.maxPower(limits.getMaxPower());
        hardware.currentAlert(limits.getCurrentAlertAmps());
    }

    public ProfiledPositionMotor(MetaMotor hardware,
                                 GoBILDAMotorTypes motorType,
                                 MotionProfilingCoefficients coefficients,
                                 double maxPower) {
        this.hardware = hardware;
        this.encoderConverter = new EncoderConverter(motorType);
        this.controller = new TrapezoidalMotionProfileController(coefficients);
        this.maxPower = maxPower;
    }

    public static ProfiledPositionMotor fromLegacyTickCoefficients(
            String hardwareName,
            GoBILDAMotorTypes motorType,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
            MotionProfilingCoefficients legacyCoefficients,
            MotorLimits limits
    ) {
        EncoderConverter encoderConverter = new EncoderConverter(motorType);
        MotionProfilingCoefficients scaledCoefficients =
                MotorCoefficientScaler.fromLegacyTickSpace(legacyCoefficients, encoderConverter);
        return new ProfiledPositionMotor(
                hardwareName,
                motorType,
                direction,
                zeroPowerBehavior,
                scaledCoefficients,
                limits
        );
    }

    public void init(HardwareMap hardwareMap) {
        hardware.init(hardwareMap);
        hardware.setTargetPosition(0);
        hardware.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetController();
    }

    public double update(LoopState loopState) {
        if (loopState.getDt() <= 0.0) {
            return hardware.getPower();
        }

        MotionState currentState = readCurrentState(loopState.getDt());
        if (!profileInitialized) {
            controller.reset(currentState);
            profileInitialized = true;
        }

        if (Math.abs(targetAngle.toRadians() - currentState.getPosition().toRadians())
                <= targetTolerance.toRadians()) {
            controller.reset(currentState);
            hardware.setPower(0.0);
            return hardware.getPower();
        }

        MotionState targetState = new MotionState(
                targetAngle,
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        );
        double output = controller.update(
                targetState,
                currentState,
                loopState.getDt(),
                loopState.getBatteryVoltageFactor()
        );
        hardware.setPower(Range.clip(output, -maxPower, maxPower));
        return hardware.getPower();
    }

    public void setTargetAngle(Angle targetAngle) {
        double clippedRadians = Range.clip(
                targetAngle.toRadians(),
                minAngle.toRadians(),
                maxAngle.toRadians()
        );
        this.targetAngle = Angle.fromRadians(clippedRadians);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngle.toRadians() - getMeasuredAngle().toRadians())
                <= targetTolerance.toRadians();
    }

    public void setTargetTolerance(Angle targetTolerance) {
        this.targetTolerance = Angle.fromRadians(Math.max(0.0, targetTolerance.toRadians()));
    }

    public void setAngleLimits(Angle minAngle, Angle maxAngle) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        setTargetAngle(targetAngle);
    }

    public Angle getMeasuredAngle() {
        return encoderConverter.ticksToAngle(hardware.getCurrentPositionTicks());
    }

    public double getMeasuredTicks() {
        return hardware.getCurrentPositionTicks();
    }

    public AngularVelocity getMeasuredVelocity() {
        return encoderConverter.ticksPerSecondToVelocity(hardware.getVelocityTicksPerSecond());
    }

    public MotionState getReferenceState() {
        return controller.getReferenceState();
    }

    public void setPower(double power) {
        resetController();
        hardware.setPower(power);
    }

    public double getPower() {
        return hardware.getPower();
    }

    public double getCurrentAmps() {
        return hardware.getCurrentAmps();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        hardware.maxPower(maxPower);
    }

    public void resetController() {
        controller.reset();
        profileInitialized = false;
        hasLastMeasurement = false;
        lastMeasuredVelocityRadPerSec = 0.0;
    }

    private MotionState readCurrentState(double dt) {
        double velocityRadPerSec =
                encoderConverter.ticksPerSecondToVelocity(hardware.getVelocityTicksPerSecond())
                        .toRadPerSec();
        double acceleration = 0.0;
        if (hasLastMeasurement) {
            acceleration = (velocityRadPerSec - lastMeasuredVelocityRadPerSec) / dt;
        }
        lastMeasuredVelocityRadPerSec = velocityRadPerSec;
        hasLastMeasurement = true;
        return new MotionState(
                encoderConverter.ticksToAngle(hardware.getCurrentPositionTicks()),
                AngularVelocity.fromRadPerSec(velocityRadPerSec),
                acceleration
        );
    }
}
