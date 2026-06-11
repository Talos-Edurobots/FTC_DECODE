# Turret Auto PID Switching & Dual Controller Tuning

## Overview
This document outlines the changes made to introduce a "double controller approach" to the Turret subsystem. The primary goal was to automatically switch from the standard `TrapezoidalMotionProfileController` into a simple `PID` controller when the turret gets very close to its target position. This was designed to be completely transparent to TeleOp and Autonomous code, meaning no architectural changes were needed outside of the `Turret.java` subsystem.

## Changes Made

### 1. `Turret.java`
- **Auto-Switching Variables**: Added `autoSwitchToPid` (boolean) and `pidSwitchThresholdDegrees` (double). These control whether the automatic transition happens and at what distance threshold it activates (defaulting to 5.0 degrees).
- **Dual Tuning Sets**: Created separate configurable coefficients for the manual PID mode (`manualKp`, `manualKi`, `manualKd`, `manualKs`). Previously, the `applyManualPositionPid` method reused the profiled control coefficients, which could restrict fine-tuning.
- **Dynamic Switching Logic**: Modified `loop()` to evaluate the absolute error in degrees. If it falls within the `pidSwitchThresholdDegrees`, it transitions the `controlMode` from `PROFILED` to `MANUAL_PID` and invokes `applyManualPositionPid`.
- **Derivative Kickback Prevention**: When swapping to `MANUAL_PID`, `lastManualError` is seeded with the current `errorDeg`. This prevents the derivative term from spiking drastically due to measuring `(errorDeg - 0.0) / dt` on the first cycle.

### 2. `Debugger.java`
- **`KaTestOpMode` Update**: Brought the new `manualKp`, `manualKi`, `manualKd`, and `manualKs` parameters into the `KaTestOpMode` scope. The opmode now safely reads these on initialization, maps them to the `@Configurable` panel for dynamic tuning, continuously applies them, and restores their original values upon stopping.
- **`TurretStickTeleOp` Functionality**: Because this opmode uses `PanelsConfigurables.INSTANCE.refreshClass(Turret.class)`, the newly added static fields in `Turret.java` automatically display in the tuning dashboard, providing immediate support for tuning the dual-controller logic.

## Strategy & Achievements
The strategy behind these changes was to optimize the "last mile" aiming precision of the turret. 
Trapezoidal profiling is excellent for rapid, smooth traversal across large gaps while respecting velocity and acceleration limits. However, near the very end of a motion, standard PID can provide a sharper, more responsive "snap" onto a target position (or adapt to Limelight micro-adjustments more fluidly).

By splitting the PID coefficients into two separate tuning sets, you gain the ability to heavily damp the profiled approach for smooth transit while sharpening the manual PID approach for aggressive final-degree targeting. Furthermore, implementing the logic strictly inside the `loop()` of `Turret.java` means all high-level commands (`lookToGoal`, `setAngleRadians`) inherently inherit this improved behavior without needing separate "fine tune" commands in OpModes.
