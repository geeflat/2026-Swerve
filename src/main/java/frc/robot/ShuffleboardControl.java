package frc.robot;

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
    private static int nextColumn;
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Motor Controls");

    // List of all registered motors for easy updating
    private static final List<MotorControlGroup> motorGroups = new ArrayList<>();
    private static GenericEntry emergencyStopEntry;
    private static final String EMERGENCY_KEY = "Emergency Stop All";

    // Called once from RobotContainer (e.g., in constructor or robotInit)
    public static void setupDashboard() {

        nextColumn = 0;

        emergencyStopEntry = Shuffleboard.getTab("Motor Controls")
            .add(EMERGENCY_KEY, false)  // Starts as off/false
            .withWidget(BuiltInWidgets.kToggleButton)
            .withProperties(Map.of(
                "true color", "#FF0000",      // Red when active
                "false color", "#00FF00"      // Green when safe
            ))
            .withPosition(0, 0)       // Top-left corner
            .withSize(4, 2)                   // Make it big and prominent
            .getEntry();

        // Optional: Add a label or note above/beside it
        Shuffleboard.getTab("Motor Controls")
            .add("E-STOP WARNING", "PRESS TO KILL ALL MOTORS")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5, 0)
            .withSize(4, 1);
    }

    /**
     * Register a motor that runs continuously (power % control).
     */
    public static void registerContinuousMotor(
            String name,           // e.g. "Intake Roller"
            int deviceId,          // CAN ID
            String bus,            // "rio" or "canivore"
            MotorAccessor motor    // Interface to get/set the motor
    ) {
        ShuffleboardLayout layout = createMotorLayout(name);

        GenericEntry powerSetpoint = layout.add("Power", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0, "block increment", 0.05))
            .withSize(3, 5)
            .withPosition(0 + (nextColumn * 4), 3)
            .getEntry();

        GenericEntry enabled = layout.add("Enabled", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withSize(3, 5)
                .withPosition(0 + (nextColumn * 4), 3)
                .getEntry();

        GenericEntry currentPower = layout.add("Current Power", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -1.0, "max", 1.0))
                .withSize(3, 5)
                .withPosition(0 + (nextColumn * 4), 3)
                .getEntry();

        motorGroups.add(new MotorControlGroup(
                motor, powerSetpoint, enabled, currentPower, null, null, null, true));

        nextColumn++;
    }

    /**
     * Register a motor that runs to position (e.g. arm, elevator, shooter hood).
     */
    public static void registerPositionMotor(
            String name,
            int deviceId,
            String bus,
            MotorAccessor motor
    ) {
        ShuffleboardLayout layout = createMotorLayout(name);

        GenericEntry positionSetpoint = layout.add("Position Setpoint", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1.0, "max", 1.0, "block increment", 0.05))
                .withSize(3, 5)
                .withPosition(0 + (nextColumn * 4), 3)
                .getEntry();

        GenericEntry goButton = layout.add("Go to Position", false)
                .withWidget(BuiltInWidgets.kToggleButton)  // or use a command button if preferred
                .withSize(3, 5)
                .withPosition(0  + (nextColumn * 4),3)
                .getEntry();

        GenericEntry currentPosition = layout.add("Current Position", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(3, 5)
                .withPosition(0 + (nextColumn * 4), 3)
                .getEntry();

        motorGroups.add(new MotorControlGroup(
                motor, null, null, null, positionSetpoint, goButton, currentPosition, false));

        nextColumn++;
    }

    private static ShuffleboardLayout createMotorLayout(String name) {
        return tab.getLayout(name, BuiltInLayouts.kList)
                .withSize(3, 5)
                .withPosition(motorGroups.size() * 4, 0)  // Auto-place horizontally; adjust as needed
                .withProperties(Map.of("Label position", "TOP"));
    }

    /**
     * Call this from RobotContainer.teleopPeriodic() or a dedicated command.
     */
    public static void update() {
        boolean eStopActive = emergencyStopEntry.getBoolean(false);

        if (eStopActive) { // Force all motors to safe state (0 power, disabled)
            for (MotorControlGroup group : motorGroups) {
                group.emergencyStop();  // We'll add this method below
            }
        } else { // Normal operation
            for (MotorControlGroup group : motorGroups) {
                group.update();
            }
        }
    }

    // ------------------------------------------------------------------------
    // Internal class to hold one motor's controls and logic
    private static class MotorControlGroup {
        private final MotorAccessor motor;
        private final GenericEntry powerSetpoint;
        private final GenericEntry enabled;
        private final GenericEntry currentPower;
        private final GenericEntry positionSetpoint;
        private final GenericEntry goButton;
        private final GenericEntry currentPosition;
        private final boolean isContinuous;
        private double lastAppliedPower = 0.0;
        private double lastAppliedPosition = 0.0;
        private double lastSetpoint = 0.0;

        MotorControlGroup(MotorAccessor motor,
                          GenericEntry powerSetpoint, GenericEntry enabled, GenericEntry currentPower,
                          GenericEntry positionSetpoint, GenericEntry goButton, GenericEntry currentPosition,
                          boolean isContinuous) {
            this.motor = motor;
            this.powerSetpoint = powerSetpoint;
            this.enabled = enabled;
            this.currentPower = currentPower;
            this.positionSetpoint = positionSetpoint;
            this.goButton = goButton;
            this.currentPosition = currentPosition;
            this.isContinuous = isContinuous;
            this.lastAppliedPower = 0.0;
            this.lastAppliedPosition = 0.0;
        }
        void emergencyStop() {
            motor.setPower(0.0);  // Or motor.set(0.0) if using unified set()
            // If position mode, perhaps call motor.stop() or set to current pos
            if (currentPower != null) {
                currentPower.setDouble(0.0);
            }
            if (enabled != null) {
                enabled.setBoolean(false);  // Disable individual toggle too
            }
        }
    // void update() {
    //     if (isContinuous) {
    //         boolean isEnabled = enabled.getBoolean(false);
    //         double power = isEnabled ? powerSetpoint.getDouble(0.0) : 0.0;
    //         motor.setPower(power);
    //         currentPower.setDouble(power);  // Or better: motor.getPower() if your accessor provides it
    //     } else {
    //         if (goButton.getBoolean(false)) {
    //             double target = positionSetpoint.getDouble(0.0);
    //             motor.setPosition(target);
    //             goButton.setBoolean(false);  // Reset button after triggering
    //         }
    //         currentPosition.setDouble(motor.getPosition());  // ← Now this will resolve
    //     }
    // }
        void update() {
            if (isContinuous) {
                boolean currentlyEnabled = enabled.getBoolean(false);
                double currentSetpoint = powerSetpoint.getDouble(0.0);

                if (currentlyEnabled) {
                // Detect if setpoint changed since last update (approx while enabled)
                if (Math.abs(currentSetpoint - lastSetpoint) > 0.001) {  // Use epsilon for floating point
                // Change detected while enabled → auto-stop and disable
                    motor.setPower(0.0);
                    currentPower.setDouble(0.0);
                    enabled.setBoolean(false);
                    lastAppliedPower = 0.0;  // Reset applied tracking
                } else {
                // No change → apply and track
                    motor.setPower(currentSetpoint);
                    currentPower.setDouble(motor.getPower());  // Or currentSetpoint if no get
                    lastAppliedPower = currentSetpoint;
                }
                } else {
                    motor.setPower(0.0);
                    currentPower.setDouble(0.0);
                    // Do not reset lastApplied when disabled — allows re-enable to new value if changed while off
                }
            lastSetpoint = currentSetpoint;  // Always update for next cycle
            } else {
        // Position mode (similar logic)
            boolean goPressed = goButton.getBoolean(false);
            double currentSetpoint = positionSetpoint.getDouble(0.0);
                if (goPressed) {
                    if (Math.abs(currentSetpoint - lastSetpoint) > 0.001) {
                        // Change while "go" active → cancel
                        goButton.setBoolean(false);
                        // Optional: motor.stop();
                    } else {
                        motor.setPosition(currentSetpoint);
                        goButton.setBoolean(false);  // Reset after apply
                        lastAppliedPosition = currentSetpoint;
                    }
                }
                currentPosition.setDouble(motor.getPosition());

                lastSetpoint = currentSetpoint;
            }
        }   
    }

    // ------------------------------------------------------------------------
    // Interface that subsystems implement or provide for their motors
    public interface MotorAccessor {
        void setPower(double power);
        double getPower();
        void setPosition(double position);
        double getPosition();
    }
}