package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorCoefficientScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers.PIDFFVelocityController;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;

/**
 * Facade for motor-shaft velocity control.
 *
 * <p>This type owns the hardware adapter, encoder conversion, velocity target,
 * and per-loop state needed to run {@link PIDFFVelocityController}.
 */
public class VelocityControlledMotor {
    private final MetaMotor hardware;
    private final EncoderConverter encoderConverter;
    private final PIDFFVelocityController controller;

    private AngularVelocity targetVelocity = AngularVelocity.fromRadPerSec(0.0);
    private double lastMeasuredVelocityRadPerSec = 0.0;
    private boolean hasLastMeasurement = false;

    public VelocityControlledMotor(String hardwareName,
                                   GoBILDAMotorTypes motorType,
                                   DcMotorSimple.Direction direction,
                                   DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                                   PIDFFCoefficients coefficients,
                                   MotorLimits limits) {
        this(hardwareName, motorType, direction, zeroPowerBehavior, coefficients, limits, 0.0);
    }

    public VelocityControlledMotor(String hardwareName,
                                   GoBILDAMotorTypes motorType,
                                   DcMotorSimple.Direction direction,
                                   DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                                   PIDFFCoefficients coefficients,
                                   MotorLimits limits,
                                   double powerWriteEpsilon) {
        this(new MetaMotor(), motorType, coefficients);
        hardware.hwName(hardwareName);
        hardware.direction(direction);
        hardware.zeroPowerBehavior(zeroPowerBehavior);
        hardware.maxPower(limits.getMaxPower());
        hardware.currentAlert(limits.getCurrentAlertAmps());
        hardware.powerWriteEpsilon(powerWriteEpsilon);
    }

    public VelocityControlledMotor(MetaMotor hardware,
                                   GoBILDAMotorTypes motorType,
                                   PIDFFCoefficients coefficients) {
        this.hardware = hardware;
        this.encoderConverter = new EncoderConverter(motorType);
        this.controller = new PIDFFVelocityController(coefficients);
    }

    public static VelocityControlledMotor fromLegacyTickCoefficients(
            String hardwareName,
            GoBILDAMotorTypes motorType,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
            PIDFFCoefficients legacyCoefficients,
            MotorLimits limits
    ) {
        return fromLegacyTickCoefficients(
                hardwareName,
                motorType,
                direction,
                zeroPowerBehavior,
                legacyCoefficients,
                limits,
                0.0
        );
    }

    public static VelocityControlledMotor fromLegacyTickCoefficients(
            String hardwareName,
            GoBILDAMotorTypes motorType,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
            PIDFFCoefficients legacyCoefficients,
            MotorLimits limits,
            double powerWriteEpsilon
    ) {
        EncoderConverter encoderConverter = new EncoderConverter(motorType);
        PIDFFCoefficients scaledCoefficients =
                MotorCoefficientScaler.fromLegacyTickSpace(legacyCoefficients, encoderConverter);
        return new VelocityControlledMotor(
                hardwareName,
                motorType,
                direction,
                zeroPowerBehavior,
                scaledCoefficients,
                limits,
                powerWriteEpsilon
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

        double measuredVelocityTicks = hardware.getVelocityTicksPerSecond();
        double measuredVelocityRadPerSec =
                encoderConverter.ticksPerSecondToVelocity(measuredVelocityTicks).toRadPerSec();
        double measuredAcceleration = 0.0;
        if (hasLastMeasurement) {
            measuredAcceleration =
                    (measuredVelocityRadPerSec - lastMeasuredVelocityRadPerSec) / loopState.getDt();
        }
        lastMeasuredVelocityRadPerSec = measuredVelocityRadPerSec;
        hasLastMeasurement = true;

        MotionState target = new MotionState(
                Angle.fromRadians(0.0),
                targetVelocity,
                0.0
        );
        MotionState current = new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(measuredVelocityRadPerSec),
                measuredAcceleration
        );

        double output = controller.update(
                target,
                current,
                loopState.getDt(),
                loopState.getBatteryVoltageFactor()
        );
        hardware.setPower(output);
        return hardware.getPower();
    }

    public void setTargetVelocity(AngularVelocity targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetVelocityTicksPerSecond(double ticksPerSecond) {
        this.targetVelocity = encoderConverter.ticksPerSecondToVelocity(ticksPerSecond);
    }

    public AngularVelocity getMeasuredVelocity() {
        return encoderConverter.ticksPerSecondToVelocity(hardware.getVelocityTicksPerSecond());
    }

    public double getMeasuredVelocityTicksPerSecond() {
        return hardware.getVelocityTicksPerSecond();
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetVelocityTicksPerSecond() {
        return encoderConverter.velocityToTicksPerSecond(targetVelocity);
    }

    public void setPower(double power) {
        hardware.setPower(power);
    }

    public double getPower() {
        return hardware.getPower();
    }

    public double getCurrentAmps() {
        return hardware.getCurrentAmps();
    }

    public void resetController() {
        controller.reset();
        hasLastMeasurement = false;
        lastMeasuredVelocityRadPerSec = 0.0;
    }

    public MetaMotor getHardware() {
        return hardware;
    }
}
