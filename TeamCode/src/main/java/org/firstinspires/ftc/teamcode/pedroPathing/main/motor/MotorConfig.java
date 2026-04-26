package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Deprecated
/**
 * Legacy all-in-one motor abstraction.
 *
 * <p>This class currently acts as:
 *
 * <p>- hardware wrapper around {@code DcMotorEx}
 * <p>- target container for position/velocity goals
 * <p>- control-mode switchboard
 * <p>- PID/PIDF implementation
 * <p>- trapezoidal motion-profile implementation
 * <p>- safety/limit holder
 * <p>- telemetry/debug publisher
 *
 * <p>That made it convenient to use across subsystems, but also made it hard to
 * reason about, test, and evolve. The long-term direction is to replace this
 * with smaller pieces:
 *
 * <p>- {@code MetaMotor} for raw hardware access
 * <p>- controller classes for control math only
 * <p>- explicit state objects such as {@code MotionState} / {@code LoopState}
 * <p>- subsystem or facade-level mode switching
 *
 * <p>When touching this class, prefer documentation and stabilization over adding
 * new responsibilities.
 */
public class MotorConfig {

    /* ---------------- Telemetry ---------------- */

    private final TelemetryManager telemetryM =
            PanelsTelemetry.INSTANCE.getTelemetry();

    /* ---------------- Hardware ---------------- */

    private final String hardwareName;

    public String getHardwareName() {
        return hardwareName;
    }
    private final GoBILDAMotorTypes motorType;

    /* --------------- Getters and Setters ------------*/
    public MotorConfig setDirection(DcMotor.Direction direction) {
        this.direction = direction;
        return this;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    private DcMotor.Direction direction;
    public MotorConfig setMotorUse(MotorUse motorUse) {
        this.motorUse = motorUse;
        return this;
    }
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior =
            DcMotor.ZeroPowerBehavior.FLOAT;

    public MotorMode getMotorMode() {
        return motorMode;
    }

    private MotorMode motorMode = MotorMode.OPEN_LOOP;

    private final MetaMotor motor = new MetaMotor();

    public MotorUse getMotorUse() {
        return motorUse;
    }
    public GoBILDAMotorTypes getMotorType() {
        return motorType;
    }
    public MotorConfig setMotorMode(MotorMode mode) {
        this.motorMode = mode;
        return this;
    }

    /* ---------------- Shared loop data ---------------- */

    // NOTE: These are static, so every MotorConfig instance shares the same dt and
    // battery voltage. This is one of the main architectural issues in the class.
    // In the modular rewrite, loop context should be passed explicitly per update.

    private static double batteryVoltage = 12.0;

    private static double dt = 0.0;

    public static void setBatteryVoltage(double voltage) {
        batteryVoltage = voltage;
    }

    public static void setDt(double deltaTime) {
        dt = deltaTime;
    }

    /* ---------------- Configuration ---------------- */

    private double externalGearRatio = 1.0;
    public double extraPower = 0.0;

    private MotorUse motorUse = MotorUse.FREE_SPIN;

    public void setCurrentAlert(double currentAlert) {
        this.currentAlert = currentAlert;
        motor.setCurrentAlert(currentAlert);
    }

    double currentAlert = 0;
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    /* ---------------- Control gains ---------------- */

    public double kP, kI, kD;
    public double kS, kV, kA;

    /* ---------------- Motion profile state (TICKS) ---------------- */

    // NOTE: Profile reference state lives inside this legacy facade. In the cleaner
    // design, profile-internal state should belong to the profile controller itself,
    // not to the generic motor wrapper.

    public double getTargetPositionTicks() {
        return targetPositionTicks;
    }

    private double targetPositionTicks = 0.0;

    public double getvRef() {
        return vRef;
    }

    public double getxRef() {
        return xRef;
    }

    public double getaRef() {
        return aRef;
    }

    private double xRef = 0.0;   // ticks
    private double vRef = 0.0;   // ticks / sec
    private double aRef = 0.0;   // ticks / sec^2

    public double maxVelocity = 1500.0;      // ticks / sec
    public double maxAcceleration = 3000.0;  // ticks / sec^2
    public double maxPower = 1.0;
    public double rampPowerAcceleration = 1;

    /* ---------------- Velocity PID ---------------- */

    private double targetVelocityTicks = 0.0;
    private double lastVelocityError = 0.0;
    private double velocityIntegral = 0.0;

    /* ---------------- Angle limits (radians, API only) ---------------- */

    // NOTE: These "angle" limits are mechanism-facing semantics layered onto a
    // generic motor class. For future code, prefer keeping mechanism-space limits
    // in the subsystem layer and keeping the motor package motor-shaft-centric.

    public MotorConfig setMinAngleTicks(double minAngleTicks) {
        this.minAngleTicks = minAngleTicks;
        return this;
    }

    public MotorConfig setMaxAngleTicks(double maxAngleTicks) {
        this.maxAngleTicks = maxAngleTicks;
        return this;
    }

    public double getMinAngleTicks() {
        return minAngleTicks;
    }

    public double getMaxAngleTicks() {
        return maxAngleTicks;
    }

    public MotorConfig setMinAngleRadians(double minAngleRadians) {
        this.minAngleTicks = minAngleRadians * motorType.getTicksPerRadian() * externalGearRatio;
        return this;
    }
    public MotorConfig setMaxAngleRadians(double maxAngleRadians) {
        this.maxAngleTicks = maxAngleRadians * motorType.getTicksPerRadian() * externalGearRatio;
        return this;
    }

    private double minAngleTicks = Double.NEGATIVE_INFINITY;
    private double maxAngleTicks = Double.POSITIVE_INFINITY;

    /* ---------------- Constructors ---------------- */

    public MotorConfig(String hardwareName,
                       GoBILDAMotorTypes motorType,
                       DcMotor.Direction direction) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.direction = direction;
    }

    public MotorConfig(String hardwareName,
                       GoBILDAMotorTypes motorType) {
        this(hardwareName, motorType, DcMotor.Direction.FORWARD);
    }
    public MotorConfig(
            String hardwareName,
            GoBILDAMotorTypes motorType,
            DcMotor.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        this(hardwareName, motorType, direction);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }


    /* ---------------- Initialization ---------------- */

    // NOTE: init() currently decides FTC run modes based on the legacy MotorMode
    // enum. During migration, this logic should gradually move to smaller,
    // purpose-built motor facades instead of a single universal switch.

    public com.qualcomm.robotcore.hardware.DcMotorEx init(HardwareMap hardwareMap) {
        motor.hwName(hardwareName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.maxPower(maxPower);
        if (currentAlert != 0) {
            motor.setCurrentAlert(currentAlert);
        }
        motor.init(hardwareMap);
        if (motorMode == MotorMode.SIMPLE_POSITION) {
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (motorMode == MotorMode.VELOCITY_CONTROL ||
                motorMode == MotorMode.PROFILED_PIDF) {
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return motor.getMotor();
    }

    /* ---------------- Basic accessors ---------------- */

    // These methods expose raw motor values directly from the SDK motor. They are
    // the least controversial part of this class and are good candidates to move
    // first into MetaMotor / a hardware adapter.

    public void setPower(double power) {
        motor.maxPower(maxPower);
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }
    public double getVelocity() {
        return motor.getVelocityTicksPerSecond();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPositionTicks();
    }

    public double getCurrent() {
        return motor.getCurrentAmps();
    }

    /* ---------------- Configuration setters ---------------- */

    // This section mixes physical configuration, controller gains, and output
    // limits. In the rewrite, these concerns should be split into dedicated data
    // classes rather than kept as mutable fields on one giant object.

    public MotorConfig addExternalGearRatio(double ratio) {
        this.externalGearRatio *= ratio;
        return this;
    }

    public MotorConfig setPIDFCoefficients(double kP, double kI, double kD,
                                           double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public MotorConfig setMotionProfileCoefficients(double maxVel,
                                                    double maxAccel,
                                                    double maxPower) {
        this.maxVelocity = maxVel;
        this.maxAcceleration = maxAccel;
        this.maxPower = maxPower;
        return this;
    }

    /* ---------------- Position targets ---------------- */

    // NOTE: This class accepts targets in both ticks and angle units. That is handy,
    // but it also mixes motor-space and mechanism-space concerns. The rewrite should
    // make unit boundaries more explicit.

    public void setRadianLimit(double minAngle, double maxAngle) {
        setMinAngleRadians(minAngle);
        setMaxAngleRadians(maxAngle);
    }

    public void setPositionInRadians(double radians) {
        int toTicks = (int) (radians * motorType.getTicksPerRadian() * externalGearRatio);
        targetPositionTicks = Range.clip(toTicks, minAngleTicks, maxAngleTicks);
    }


    public void setPositionInTicks(double ticks) {
        targetPositionTicks = ticks;
    }
    public void setPositionInDegrees(double degrees) {
        targetPositionTicks = degrees * motorType.getTicksPerDegree() * externalGearRatio;
        telemetryM.addData("TICKS PER ROTATION", motorType.getTicksPerOutputRev());
        telemetryM.addData("SET POS DEG VAL", degrees);
        telemetryM.addData("SET POS TICKS VAL", targetPositionTicks);
    }

    /* ---------------- Profiled position PIDF ---------------- */
    public void updateSimplePositionControl() {
        // Thin wrapper over FTC RUN_TO_POSITION behavior.
        // This is useful during transition, but it does not fit especially well with
        // the rest of the custom-control architecture.
        motor.setTargetPosition((int) targetPositionTicks);
        motor.setVelocity(2500); // 2500
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
    }

    public void updatePositionProfiledPIDF() {
        // Legacy inline implementation of:
        // target -> trapezoidal profile -> PD/PIDF -> motor power
        //
        // Suggested structure:
        // - profile generation in TrapezoidalMotionProfileController
        // - control math in a controller class
        // - power application in a hardware/facade layer
        if (dt <= 0.0) return;
        double position = motor.getCurrentPositionTicks();
        double velocity = motor.getVelocityTicksPerSecond();

        double remaining = targetPositionTicks - xRef;

        double stoppingDistance =
                (vRef * vRef) / (2.0 * maxAcceleration);

        if (Math.abs(remaining) <= stoppingDistance) {
            aRef = -Math.signum(velocity) * maxAcceleration;
        } else {
            aRef = Math.signum(remaining) * maxAcceleration;
        }

        vRef += aRef * dt;
        vRef = Range.clip(vRef, -maxVelocity, maxVelocity);

        xRef += vRef * dt;

        if (Math.signum(targetPositionTicks - xRef)
                != Math.signum(remaining)) {
            xRef = targetPositionTicks;
            vRef = 0.0;
            aRef = 0.0;
        }

        double positionError = xRef - position;
        double velocityError = vRef - velocity;

        double pidVolts =
                kP * positionError +
                        kD * velocityError;

        double ffVolts =
                kS * Math.signum(vRef) +
                        kV * vRef +
                        kA * aRef;

        double power =
                (pidVolts + ffVolts) / batteryVoltage;

        motor.maxPower(maxPower);
        motor.setPower(power);
        telemetryM.addData("Ks", kS);
        telemetryM.addData("battery", batteryVoltage);
        telemetryM.addData("power", power);
        telemetryM.addData("pid volts", pidVolts);
        telemetryM.addData("ff volts", ffVolts);
        telemetryM.addData("profiled pidf dt", dt);
        telemetryM.addData("reset condition", Math.signum(targetPositionTicks - xRef)
                != Math.signum(remaining));
        telemetryM.addData("aref", aRef);
        telemetryM.addData("pos error", remaining);
        telemetryM.addData("is at deceleration state", Math.abs(remaining) <= stoppingDistance);
    }

    public double rampPower(double current, double target, double dt) {
        // Output slew-rate limiting. If this behavior remains useful, it likely
        // deserves its own clearly named helper / policy object in the rewrite.
        double maxPowerChange = rampPowerAcceleration * dt;
        double diff = target - current;
        if (Math.abs(diff) > maxPowerChange) {
            diff = Math.signum(diff) * maxPowerChange;
        }
        return current + diff;
    }
    public void manualPositionPIDF(double error) {
        // Custom closed-loop path used by some mechanisms where the subsystem
        // computes its own error and delegates only the motor output stage here.
        // This is another sign that multiple different abstractions were folded
        // into one class.
        if (dt == 0) {
            throw new ArithmeticException("dt cannot be 0");
        }
        double derivative =
                (error - lastVelocityError) / dt;
        lastVelocityError = error;

        double output = rampPower( motor.getPower(),
                kP * error +
                        kI * velocityIntegral +
                        kD * derivative + kS * Math.signum(error),
                dt);
        int pos = motor.getCurrentPositionTicks();
        boolean motorTooLowPos = pos < minAngleTicks;
        boolean motorTooHighPos = pos > maxAngleTicks;
        if (Math.abs(error) < 1) {
            output = extraPower;
        }
        if (output < maxPower) {
            velocityIntegral += error;
        };
        if (motorTooLowPos && output < 0) {
            output = 0;
            telemetryM.addLine("turret motor pos too low");
        }
        if (motorTooHighPos && output > 0) {
            output = 0;
            telemetryM.addLine("turret motor pos too high");
        }
        telemetryM.addData("output", output);
        telemetryM.addData("extra power 2", extraPower);
        telemetryM.addData("error", error);
        telemetryM.addData("pos", pos);
        telemetryM.addData("too low", motorTooLowPos);
        telemetryM.addData("too high", motorTooHighPos);
        motor.maxPower(maxPower);
        motor.setPower(output);
    }

    /* ---------------- Velocity PIDF ---------------- */

    // This path reads measured velocity directly from the SDK motor and computes
    // output inline. In the rewrite, this should map naturally to:
    // MetaMotor + PIDFFVelocityController + a small velocity motor facade.

    public void setVelocityTicksPerSecond(double ticksPerSecond) {
        targetVelocityTicks = ticksPerSecond;
    }

    public void updateVelocityPIDF() {
        if (dt <= 0.0) return;

        double velocity = motor.getVelocityTicksPerSecond();
        double error = targetVelocityTicks - velocity;

        velocityIntegral += error * dt;
        double derivative =
                (error - lastVelocityError) / dt;
        lastVelocityError = error;

        double output =
                kP * error +
                        kI * velocityIntegral +
                        kD * derivative +
                        (kS * Math.signum(targetVelocityTicks)
                                + kV * targetVelocityTicks)
                                / batteryVoltage;

        motor.maxPower(maxPower);
        motor.setPower(Range.clip(output, 0, maxPower));
        telemetryM.addData("shooter output from update", output);
        telemetryM.addData("kp", kP);
        telemetryM.addData("max power", maxPower);
        telemetryM.addData("error", error);
    }
    public void update() {
        // Legacy mode switch. This central branching is convenient, but it is also
        // the reason the class keeps accumulating unrelated responsibilities.
        // Long-term direction: replace with separate use-case-specific motor types.
        if (motorMode == MotorMode.VELOCITY_CONTROL) {
            updateVelocityPIDF();
        } else if (motorMode == MotorMode.PROFILED_PIDF) {
            updatePositionProfiledPIDF();
        } else if (motorMode == MotorMode.SIMPLE_POSITION) {
            updateSimplePositionControl();
        } else if (motorMode == MotorMode.OPEN_LOOP) {

        }
    }
}
