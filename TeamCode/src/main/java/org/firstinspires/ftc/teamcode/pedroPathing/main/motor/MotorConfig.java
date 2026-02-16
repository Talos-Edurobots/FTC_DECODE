package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorConfig {

    /* ---------------- Telemetry ---------------- */

    private final TelemetryManager telemetryM =
            PanelsTelemetry.INSTANCE.getTelemetry();

    /* ---------------- Hardware ---------------- */

    private final String hardwareName;

    public String getHardwareName() {
        return hardwareName;
    }
    private final GoBildaMotor motorType;

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

    private DcMotorEx motor;

    public MotorUse getMotorUse() {
        return motorUse;
    }
    public GoBildaMotor getMotorType() {
        return motorType;
    }
    public MotorConfig setMotorMode(MotorMode mode) {
        this.motorMode = mode;
        return this;
    }

    /* ---------------- Shared loop data ---------------- */

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

    /* ---------------- Control gains ---------------- */

    public double kP, kI, kD;
    public double kS, kV, kA;

    /* ---------------- Motion profile state (TICKS) ---------------- */

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

    /* ---------------- Velocity PID ---------------- */

    private double targetVelocityTicks = 0.0;
    private double lastVelocityError = 0.0;
    private double velocityIntegral = 0.0;

    /* ---------------- Angle limits (radians, API only) ---------------- */

    public MotorConfig setMinAngleTicks(double minAngleTicks) {
        this.minAngleTicks = minAngleTicks;
        return this;
    }

    public MotorConfig setMaxAngleTicks(double maxAngleTicks) {
        this.maxAngleTicks = maxAngleTicks;
        return this;
    }

    private double minAngleTicks = Double.NEGATIVE_INFINITY;
    private double maxAngleTicks = Double.POSITIVE_INFINITY;

    /* ---------------- Constructors ---------------- */

    public MotorConfig(String hardwareName,
                       GoBildaMotor motorType,
                       DcMotor.Direction direction) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.direction = direction;
    }

    public MotorConfig(String hardwareName,
                       GoBildaMotor motorType) {
        this(hardwareName, motorType, DcMotor.Direction.FORWARD);
    }
    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotor.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        this(hardwareName, motorType, direction);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }


    /* ---------------- Initialization ---------------- */

    public DcMotorEx init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, hardwareName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        if (motorMode == MotorMode.SIMPLE_POSITION) {
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        return motor;
    }

    /* ---------------- Basic accessors ---------------- */

    public void setPower(double power) {
        motor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public double getPower() {
        return motor.getPower();
    }
    public double getVelocity() {
        return motor.getVelocity();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /* ---------------- Configuration setters ---------------- */

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

    public void setRadianLimit(double minAngle, double maxAngle) {
        this.minAngleTicks = minAngle;
        this.maxAngleTicks = maxAngle;
    }

    public void setPositionInRadians(double radians) {
        double clamped =
                Range.clip(radians, minAngleTicks, maxAngleTicks);

        targetPositionTicks =
                clamped * motorType.getTicksPerRadian() * externalGearRatio;
    }


    public void setPositionInTicks(double ticks) {
        targetPositionTicks = ticks;
    }
    public void setPositionInDegrees(double degrees) {
        double radians = Math.toRadians(degrees);
        setPositionInRadians(radians);
    }

    /* ---------------- Profiled position PIDF ---------------- */
    public void updateSimplePositionControl() {
        motor.setTargetPosition((int) targetPositionTicks);
        motor.setVelocity(2500); // 2500
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
    }

    public void updatePositionProfiledPIDF() {
        if (dt <= 0.0) return;

        double position = motor.getCurrentPosition();
        double velocity = motor.getVelocity();

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

        motor.setPower(
                Range.clip(power, -maxPower, maxPower)
        );
        telemetryM.addData("Ks", kS);
        telemetryM.addData("battery", batteryVoltage);
        telemetryM.addData("power", power);
        telemetryM.addData("pid volts", pidVolts);
        telemetryM.addData("ff volts", ffVolts);
    }

    public void manualPositionPIDF(double error) {
        if (dt == 0) {
            throw new ArithmeticException("dt cannot be 0");
        }
        double derivative =
                (error - lastVelocityError) / dt;
        lastVelocityError = error;

        double output =
                kP * error +
                        kD * derivative;
        int pos = motor.getCurrentPosition();
        boolean motorTooLowPos = pos < minAngleTicks;
        boolean motorTooHighPos = pos > maxAngleTicks;
        if (Math.abs(error) < 1) {
            output = 0;
        }
        output += extraPower;
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
        motor.setPower(
                Range.clip(output, -maxPower, maxPower)
        );
    }

    /* ---------------- Velocity PIDF ---------------- */

    public void setVelocityTicksPerSecond(double ticksPerSecond) {
        targetVelocityTicks = ticksPerSecond;
    }

    public void updateVelocityPIDF() {
        if (dt <= 0.0) return;

        double velocity = motor.getVelocity();
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

        motor.setPower(
                Range.clip(output, -maxPower, maxPower)
        );
    }
    public void update() {
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
