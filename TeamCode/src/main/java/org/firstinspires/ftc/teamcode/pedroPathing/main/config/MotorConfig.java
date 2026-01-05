package org.firstinspires.ftc.teamcode.pedroPathing.main.config;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorConfig {
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final String hardwareName;
    private final GoBildaMotor motorType;
    private DcMotor.Direction direction;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private double externalGearRatio = 1.0;
    private DcMotorEx motor;
    private HardwareMap hwMap;

    private MotorUse motorUse = MotorUse.FREE_SPIN;

    public double kP, kI, kD, kS, kU, kA;
    private double targetPositionRadians = 0;
    private double targetVelocityTicks = 0;
    private double lastPosition = 0;
    private double lastVelocity = 0;
    private double integralVelocitySum = 0;
    private double xRef;   // position reference
    private double vRef;   // velocity reference
    private double aRef;   // acceleration reference
    // Motion constraints
    public static double maxVelocity = 1500;      // ticks/sec
    public static double maxAcceleration = 3000;  // ticks/sec^2
    public static double maxPower = 1.0;
    private static int position = 0;
    private double minAngle = Double.NEGATIVE_INFINITY;
    private double maxAngle = Double.POSITIVE_INFINITY;

    public double getMinAngle() {
        return minAngle;
    }
    public double getMaxAngle() {
        return maxAngle;
    }

    public void setRadianLimit(double minAngle, double maxAngle) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }


    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotor.Direction direction
    ) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.direction = direction;
    }

    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType
    ) {
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
    public MotorConfig(
            String hardwareName,
            GoBildaMotor motorType,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
            DcMotor.RunMode runMode
    ) {
        this(hardwareName, motorType, direction);
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
    }

    public MotorUse getMotorUse() {
        return motorUse;
    }

    public MotorConfig setMotorUse(MotorUse motorUse) {
        this.motorUse = motorUse;
        return this;
    }

    public String getHardwareName() {
        return hardwareName;
    }

    public GoBildaMotor getMotorType() {
        return motorType;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    /** Initializes and configures the motor from hardwareMap */
    public DcMotorEx init(HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, hardwareName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        return motor;
    }

    public MotorConfig setExternalGearRatio(double externalGearRatio) {
        this.externalGearRatio = externalGearRatio;
        return this;
    }
    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
        motor.setDirection(direction);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getVelocity(boolean useExternalGearRatio) {
        return useExternalGearRatio ? getVelocity() * externalGearRatio : getVelocity();
    }
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    public int getCurrentPosition() {
        return position;
    }
    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }
    public double getPower() {
        return motor.getPower();
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public MotorConfig setPIDFCoefficients(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kU = kV;
        this.kA = kA;
        return this;
    }

    public MotorConfig setMotionProfileCoefficients(double maxVelocity, double maxAcceleration, double maxPower) {
        MotorConfig.maxVelocity = maxVelocity;
        MotorConfig.maxAcceleration = maxAcceleration;
        MotorConfig.maxPower = maxPower;
        return this;
    }

    public void setPositionInTicks(double ticks) {
        setPositionInRadians(ticks / motorType.getTicksPerRadian());
    }
    public void setPositionInDegrees(double degrees) {
        double ticks = degrees / 360.0 * motorType.getTicksPerOutputRev() * externalGearRatio;
        setPositionInTicks(ticks);
    }
    public void setPositionInRadians(double radians) {
        double currentAngle = getCurrentPosition() / motorType.getTicksPerRadian();

        double bestTarget = Double.NaN;
        double minError = Double.POSITIVE_INFINITY;

        double[] candidates = {
                radians,
                radians + 2 * Math.PI,
                radians - 2 * Math.PI
        };

        for (double candidate : candidates) {
            if (candidate < minAngle || candidate > maxAngle) continue;

            double error = candidate - currentAngle;
            double absError = Math.abs(error);

            if (absError < minError) {
                minError = absError;
                bestTarget = candidate;
            }
        }

        // If no valid candidate exists, clamp (failsafe)
        if (Double.isNaN(bestTarget)) {
            bestTarget = Range.clip(currentAngle, minAngle, maxAngle);
        }
        targetPositionRadians = bestTarget;
    }
    //    public static double getBatteryVoltage(HardwareMap hwMap) {
//        return hwMap.voltageSensor.iterator().next().getVoltage();
//    }
    public void updatePositionProfiledPIDF(double dt, double batteryVoltage) {
        if (dt <= 0) return;
        position = motor.getCurrentPosition();

        double velocity = (position - lastPosition) / dt;
        lastPosition = position;

        /* -------- Trapezoidal motion profile -------- */

        double remaining = targetPositionRadians - xRef;

        // Compute stopping distance
        double stoppingDistance =
                (vRef * vRef) / (2.0 * maxAcceleration);

        // Decide acceleration direction
        if (Math.abs(remaining) <= stoppingDistance) {
            aRef = -Math.signum(vRef) * maxAcceleration;
        } else {
            aRef = Math.signum(remaining) * maxAcceleration;
        }

        // Integrate profile
        vRef += aRef * dt;
        vRef = Range.clip(vRef, -maxVelocity, maxVelocity);

        xRef += vRef * dt;

        // Prevent overshoot
        if (Math.signum(targetPositionRadians - xRef) != Math.signum(remaining)) {
            xRef = targetPositionRadians;
            vRef = 0;
            aRef = 0;
        }

        /* -------- PID (tracking error) -------- */

        double positionError = xRef - position;
        double velocityError = vRef - velocity;

        double pTerm = kP * positionError;
        double dTerm = kD * velocityError;

        /* -------- Feedforward (voltage) -------- */

        double ffVolts =
                kS * Math.signum(vRef) +
                        kU * vRef +
                        kA * aRef;

        /* -------- Combine & normalize -------- */


        double outputVolts = pTerm + dTerm + ffVolts;
        double outputPower = outputVolts / batteryVoltage;

        outputPower = Range.clip(outputPower, -maxPower, maxPower);
        motor.setPower(outputPower);
        telemetryM.addData("power", outputPower);
        telemetryM.addData("target pos", targetPositionRadians);
        telemetryM.addData("position", position);
        telemetryM.addData("position error", positionError);
        telemetryM.addData("ff volts", ffVolts);
        telemetryM.addData("v ref", vRef);
        telemetryM.update();
    }
    public void setVelocityTicksPerSecond(double ticksPerSecond) {
        targetVelocityTicks = ticksPerSecond;
    }
    public void updateVelocityPIDF(double dt, double batteryVoltage) {
        double currentVelocity = motor.getVelocity();
        double error = targetVelocityTicks - currentVelocity;
        double derivative = (error - lastVelocity) / dt;
        integralVelocitySum += error * dt;
        lastVelocity = currentVelocity;
        motor.setPower(
                kP * error +
                        kI * integralVelocitySum +
                        kD * derivative +
                        (kS * Math.signum(targetVelocityTicks) +
                        kU * targetVelocityTicks) / batteryVoltage
        );
        telemetryM.addData("error", error);
        telemetryM.addData("current vel", currentVelocity);
    }
}


