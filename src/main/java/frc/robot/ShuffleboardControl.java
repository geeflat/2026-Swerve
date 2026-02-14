package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ShuffleboardControl {

    // === Separate tabs for clean layout ===
    private static final ShuffleboardTab openLoopTab   = Shuffleboard.getTab("Open-Loop Motors");
    private static final ShuffleboardTab velocityTab   = Shuffleboard.getTab("Velocity Motors");
    private static final ShuffleboardTab positionTab   = Shuffleboard.getTab("Position Motors");
    private static final ShuffleboardTab estopTab      = Shuffleboard.getTab("E-STOP");

    private static GenericEntry emergencyStopEntry;
    private static final List<MotorControlGroup> motorGroups = new ArrayList<>();

    public enum MotorControlType {
        OPEN_LOOP_POWER,
        CLOSED_LOOP_VELOCITY,
        POSITION_CONTROL
    }

    public static void setupDashboard() {
        // Emergency stop on its own prominent tab
        emergencyStopEntry = estopTab
            .add("EMERGENCY STOP ALL", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withProperties(Map.of("true color", "#FF0000", "false color", "#00FF00"))
            .withPosition(0, 0)
            .withSize(4, 2)
            .getEntry();

        estopTab.add("WARNING", "PRESS TO KILL ALL MOTORS")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5, 0)
            .withSize(4, 1);
    }

    // ===================================================================
    // Registration methods
    // ===================================================================

    public static void registerOpenLoopMotor(String name, TalonFX motor) {
        var layout = createMotorLayout(openLoopTab, name);
        addOpenLoopControls(layout, motor);
    }

    public static void registerVelocityMotor(String name, TalonFX motor) {
        var layout = createMotorLayout(velocityTab, name);
        addVelocityControls(layout, motor);
    }

    public static void registerPositionMotor(String name, TalonFX motor) {
        var layout = createMotorLayout(positionTab, name);
        addPositionControls(layout, motor);
    }

    // ===================================================================
    // Internal helpers
    // ===================================================================

    private static ShuffleboardLayout createMotorLayout(ShuffleboardTab tab, String name) {
        return tab.getLayout(name, BuiltInLayouts.kList)
                .withSize(3, 8)
                .withProperties(Map.of("Label position", "TOP"));
    }

    private static void addOpenLoopControls(ShuffleboardLayout layout, TalonFX motor) {
        GenericEntry power = layout.add("Power %", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1.0, "max", 1.0))
                .getEntry();

        GenericEntry enabled = layout.add("Enabled", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

        GenericEntry current = layout.add("Current Power", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .getEntry();

        motorGroups.add(new MotorControlGroup(motor, MotorControlType.OPEN_LOOP_POWER,
                enabled, power, current, null, null, null, null, null));
    }

    private static void addVelocityControls(ShuffleboardLayout layout, TalonFX motor) {
        GenericEntry rpm = layout.add("RPM Setpoint", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 6000, "block increment", 100))
                .getEntry();

        GenericEntry enabled = layout.add("Enabled", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

        GenericEntry currentRpm = layout.add("Current RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .getEntry();

        // PID fields
        GenericEntry kp = layout.add("kP", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry ki = layout.add("kI", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry kd = layout.add("kD", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry apply = layout.add("Apply PID", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withProperties(Map.of("true color", "#00FF00", "false color", "#808080"))
                .getEntry();

        motorGroups.add(new MotorControlGroup(motor, MotorControlType.CLOSED_LOOP_VELOCITY,
                enabled, rpm, currentRpm, null, kp, ki, kd, apply));
    }

    private static void addPositionControls(ShuffleboardLayout layout, TalonFX motor) {
        GenericEntry pos = layout.add("Position (deg)", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", Constants.HoodConstants.LOWER_LIMIT, "max", Constants.HoodConstants.UPPER_LIMIT))
                .getEntry();

        GenericEntry go = layout.add("Go to Pos", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

        GenericEntry currentPos = layout.add("Current Pos", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .getEntry();

        // PID fields
        GenericEntry kp = layout.add("kP", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry ki = layout.add("kI", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry kd = layout.add("kD", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        GenericEntry apply = layout.add("Apply PID", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withProperties(Map.of("true color", "#00FF00", "false color", "#808080"))
                .getEntry();

        motorGroups.add(new MotorControlGroup(motor, MotorControlType.POSITION_CONTROL,
                null, pos, currentPos, go, kp, ki, kd, apply));
    }

    public static void update() {
        boolean eStop = emergencyStopEntry.getBoolean(false);

        if (eStop) {
            motorGroups.forEach(MotorControlGroup::emergencyStop);
            return;
        }

        motorGroups.forEach(MotorControlGroup::update);
    }

    // ===================================================================
    // Motor Control Group
    // ===================================================================
    private static class MotorControlGroup {
        private final TalonFX motor;
        private final MotorControlType type;

        private final GenericEntry enabled;
        private final GenericEntry setpoint;
        private final GenericEntry currentValue;
        private final GenericEntry goButton;
        private final GenericEntry kpEntry;
        private final GenericEntry kiEntry;
        private final GenericEntry kdEntry;
        private final GenericEntry applyPidButton;

        private boolean lastApplyState = false;

        MotorControlGroup(TalonFX motor, MotorControlType type,
                          GenericEntry enabled, GenericEntry setpoint, GenericEntry currentValue,
                          GenericEntry goButton,
                          GenericEntry kp, GenericEntry ki, GenericEntry kd, GenericEntry apply) {
            this.motor = motor;
            this.type = type;
            this.enabled = enabled;
            this.setpoint = setpoint;
            this.currentValue = currentValue;
            this.goButton = goButton;
            this.kpEntry = kp;
            this.kiEntry = ki;
            this.kdEntry = kd;
            this.applyPidButton = apply;
        }

        void emergencyStop() {
            motor.set(0.0);
            if (currentValue != null) currentValue.setDouble(0.0);
            if (enabled != null) enabled.setBoolean(false);
            if (goButton != null) goButton.setBoolean(false);
            if (applyPidButton != null) applyPidButton.setBoolean(false);
        }

        void update() {
            boolean isEnabled = enabled != null && enabled.getBoolean(false);

            if (!isEnabled) {
                motor.set(0.0);
                if (currentValue != null) currentValue.setDouble(0.0);
                return;
            }

            // Normal motor control
            switch (type) {
                case OPEN_LOOP_POWER:
                    double power = setpoint.getDouble(0.0);
                    motor.set(power);
                    if (currentValue != null) currentValue.setDouble(power);
                    break;

                case CLOSED_LOOP_VELOCITY:
                    double rpm = setpoint.getDouble(0.0);
                    // TODO: Replace with your actual velocity command
                    // motor.setControl(new VelocityVoltage(Units.rotationsPerMinuteToRotationsPerSecond(rpm)));
                    break;

                case POSITION_CONTROL:
                    if (goButton != null && goButton.getBoolean(false)) {
                        double target = setpoint.getDouble(0.0);
                        // TODO: Replace with your position command
                        // motor.setPositionDegrees(target);
                        goButton.setBoolean(false);
                    }
                    break;
            }

            // PID Apply - rising edge detection (works even when disabled)
            if (applyPidButton != null) {
                boolean currentApply = applyPidButton.getBoolean(false);

                if (currentApply && !lastApplyState) {
                    double kp = kpEntry.getDouble(0.0);
                    double ki = kiEntry.getDouble(0.0);
                    double kd = kdEntry.getDouble(0.0);

                    Slot0Configs slot0 = new Slot0Configs()
                        .withKP(kp)
                        .withKI(ki)
                        .withKD(kd);

                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.Slot0 = slot0;

                    motor.getConfigurator().apply(config);

                    System.out.println("Applied PID to motor " + motor.getDeviceID() +
                                       ": kP=" + kp + ", kI=" + ki + ", kD=" + kd);

                    applyPidButton.setBoolean(false);
                }

                lastApplyState = currentApply;
            }
        }
    }
}