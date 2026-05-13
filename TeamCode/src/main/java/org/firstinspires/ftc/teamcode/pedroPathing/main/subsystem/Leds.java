package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;

@Configurable
public class Leds {
    public static double speed = 1;
    public static boolean telemetryEnabled = false;
    public static double defaultBlinkOffColor = 0;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Servo left;
    private Servo right;

    private final SideState leftState = new SideState();
    private final SideState rightState = new SideState();

    private double rgbColor = 0.28;

    public enum Side {
        LEFT,
        RIGHT,
        BOTH
    }

    public void init(HardwareMap hwmap) {
        left = hwmap.servo.get(RobotConstants.LED_LEFT);
        right = hwmap.servo.get(RobotConstants.LED_RIGHT);
        setBoth(rgbColor);
    }

    public void update(double dt) {
        updateSide(left, leftState, dt);
        updateSide(right, rightState, dt);
        addTelemetry(dt);
    }

    public void clearEffects() {
        clearEffect(Side.BOTH);
    }

    public void clearEffect(Side side) {
        if (side == Side.LEFT || side == Side.BOTH) {
            leftState.clearEffect();
            apply(left, leftState.baseColor);
        }
        if (side == Side.RIGHT || side == Side.BOTH) {
            rightState.clearEffect();
            apply(right, rightState.baseColor);
        }
    }

    public void setLeft(double val) {
        setBase(Side.LEFT, val);
    }

    public void setRight(double val) {
        setBase(Side.RIGHT, val);
    }

    public void setBoth(double val) {
        setBase(Side.BOTH, val);
    }

    public void setBase(Side side, double color) {
        if (side == Side.LEFT || side == Side.BOTH) {
            leftState.baseColor = color;
            if (!leftState.hasEffect()) {
                leftState.currentColor = color;
                apply(left, color);
            }
        }
        if (side == Side.RIGHT || side == Side.BOTH) {
            rightState.baseColor = color;
            if (!rightState.hasEffect()) {
                rightState.currentColor = color;
                apply(right, color);
            }
        }
    }

    public double getLeft() {
        return leftState.currentColor;
    }

    public double getRight() {
        return rightState.currentColor;
    }

    public double getBaseLeft() {
        return leftState.baseColor;
    }

    public double getBaseRight() {
        return rightState.baseColor;
    }

    public boolean isBusy() {
        return leftState.hasEffect() || rightState.hasEffect();
    }

    public boolean isBusy(Side side) {
        switch (side) {
            case LEFT:
                return leftState.hasEffect();
            case RIGHT:
                return rightState.hasEffect();
            case BOTH:
            default:
                return isBusy();
        }
    }

    public void showColorFor(Side side, double color, double duration) {
        showColorFor(side, color, duration, true);
    }

    public void showColorFor(Side side, double color, double duration, boolean restoreBaseAfter) {
        if (side == Side.LEFT || side == Side.BOTH) {
            startTimedHold(leftState, color, duration, restoreBaseAfter);
            apply(left, leftState.currentColor);
        }
        if (side == Side.RIGHT || side == Side.BOTH) {
            startTimedHold(rightState, color, duration, restoreBaseAfter);
            apply(right, rightState.currentColor);
        }
    }

    public void blink(Side side, double color, int count) {
        blink(side, color, defaultBlinkOffColor, 0.2, count, true);
    }

    public void blink(Side side, double color, int count, double interval) {
        blink(side, color, defaultBlinkOffColor, interval, count, true);
    }

    public void blink(Side side, double color1, double color2, double interval) {
        blink(side, color1, color2, interval, -1, false);
    }

    public void blink(Side side, double color1, double color2, double interval, int count) {
        blink(side, color1, color2, interval, count, true);
    }

    public void blink(Side side, double color1, double color2, double interval, int count, boolean restoreBaseAfter) {
        if (side == Side.LEFT || side == Side.BOTH) {
            startBlink(leftState, color1, color2, interval, count, restoreBaseAfter);
            apply(left, leftState.currentColor);
        }
        if (side == Side.RIGHT || side == Side.BOTH) {
            startBlink(rightState, color1, color2, interval, count, restoreBaseAfter);
            apply(right, rightState.currentColor);
        }
    }

    public void pulse(Side side, double color, double interval) {
        pulse(side, color, defaultBlinkOffColor, interval);
    }

    public void pulse(Side side, double color1, double color2, double interval) {
        blink(side, color1, color2, interval, -1, false);
    }

    public void alertLeft(double color, int count, double interval) {
        blink(Side.LEFT, color, defaultBlinkOffColor, interval, count, true);
    }

    public void alertRight(double color, int count, double interval) {
        blink(Side.RIGHT, color, defaultBlinkOffColor, interval, count, true);
    }

    public void alertBoth(double color, int count, double interval) {
        blink(Side.BOTH, color, defaultBlinkOffColor, interval, count, true);
    }

    public void blinkLeft(double time, double dt, double color1, double color2) {
        if (!matchesLegacyBlink(leftState, color1, color2, time)) {
            startBlink(leftState, color1, color2, time, -1, false);
        }
        updateSide(left, leftState, dt);
        addTelemetry(dt);
    }

    public void blinkRight(double time, double dt, double color1, double color2) {
        if (!matchesLegacyBlink(rightState, color1, color2, time)) {
            startBlink(rightState, color1, color2, time, -1, false);
        }
        updateSide(right, rightState, dt);
        addTelemetry(dt);
    }

    public void rgb(double dt) {
        rgbColor += dt * speed;
        if (rgbColor > .72) {
            rgbColor = .28;
        }
        setBoth(rgbColor);
    }

    private void startTimedHold(SideState state, double color, double duration, boolean restoreBaseAfter) {
        state.effectMode = EffectMode.HOLD;
        state.effectColor1 = color;
        state.effectColor2 = color;
        state.interval = Math.max(0, duration);
        state.timer = 0;
        state.remainingToggles = 1;
        state.restoreBaseAfter = restoreBaseAfter;
        applyStateColor(state, color);
    }

    private void startBlink(SideState state, double color1, double color2, double interval, int count, boolean restoreBaseAfter) {
        state.effectMode = EffectMode.BLINK;
        state.effectColor1 = color1;
        state.effectColor2 = color2;
        state.interval = Math.max(0.001, interval);
        state.timer = 0;
        state.useFirstColor = true;
        state.remainingToggles = count < 0 ? -1 : Math.max(0, count * 2 - 1);
        state.restoreBaseAfter = restoreBaseAfter;
        applyStateColor(state, color1);
    }

    private void updateSide(Servo servo, SideState state, double dt) {
        if (servo == null || !state.hasEffect()) {
            return;
        }

        state.timer += Math.max(0, dt);

        if (state.effectMode == EffectMode.HOLD) {
            if (state.timer >= state.interval) {
                finishEffect(servo, state);
            }
            return;
        }

        while (state.timer >= state.interval && state.hasEffect()) {
            state.timer -= state.interval;

            if (state.remainingToggles == 0) {
                finishEffect(servo, state);
                return;
            }

            state.useFirstColor = !state.useFirstColor;
            applyStateColor(state, state.useFirstColor ? state.effectColor1 : state.effectColor2);

            if (state.remainingToggles > 0) {
                state.remainingToggles--;
                if (state.remainingToggles == 0) {
                    finishEffect(servo, state);
                    return;
                }
            }
        }

        apply(servo, state.currentColor);
    }

    private void finishEffect(Servo servo, SideState state) {
        double finalColor = state.restoreBaseAfter ? state.baseColor : state.currentColor;
        state.clearEffect();
        state.currentColor = finalColor;
        apply(servo, finalColor);
    }

    private boolean matchesLegacyBlink(SideState state, double color1, double color2, double interval) {
        return state.effectMode == EffectMode.BLINK
                && state.remainingToggles < 0
                && state.effectColor1 == color1
                && state.effectColor2 == color2
                && state.interval == Math.max(0.001, interval);
    }

    private void applyStateColor(SideState state, double color) {
        state.currentColor = color;
    }

    private void apply(Servo servo, double color) {
        if (servo != null) {
            servo.setPosition(color);
        }
    }

    private void addTelemetry(double dt) {
        if (!telemetryEnabled) {
            return;
        }
        telemetryM.addData("led left", leftState.currentColor);
        telemetryM.addData("led right", rightState.currentColor);
        telemetryM.addData("led left timer", leftState.timer);
        telemetryM.addData("led right timer", rightState.timer);
        telemetryM.addData("led dt", dt);
    }

    private enum EffectMode {
        NONE,
        BLINK,
        HOLD
    }

    private static class SideState {
        private double baseColor = 0;
        private double currentColor = 0;
        private double effectColor1 = 0;
        private double effectColor2 = 0;
        private double interval = 0;
        private double timer = 0;
        private int remainingToggles = 0;
        private boolean useFirstColor = true;
        private boolean restoreBaseAfter = true;
        private EffectMode effectMode = EffectMode.NONE;

        private boolean hasEffect() {
            return effectMode != EffectMode.NONE;
        }

        private void clearEffect() {
            effectMode = EffectMode.NONE;
            timer = 0;
            interval = 0;
            remainingToggles = 0;
            useFirstColor = true;
            restoreBaseAfter = true;
            currentColor = baseColor;
        }
    }
}
