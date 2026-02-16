// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.ShooterContinuousCommand;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.commands.IndexToShooterCommand;
import frc.robot.commands.IntakeExtenderUp;
import frc.robot.commands.IntakeExtenderDown;
import frc.robot.Constants.BoundaryConstants;
import frc.robot.ShuffleboardControl.MotorAccessor;
import frc.robot.commands.IndexReverseCommand;
import frc.robot.commands.IndexStopCommand;
import frc.robot.commands.DynamicAimCommand;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeExtender;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimulationExtensions;
import frc.robot.subsystems.TurretSubsystem;

import frc.robot.BoundaryManager;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import frc.robot.Constants;

public class RobotContainer {

    HoodSubsystem hood;
    IndexSubsystem index;
    IntakeExtender intakeExtender;
    IntakeSubsystem intake;
    ShooterSubsystem shooter;
    TurretSubsystem turret;

    private Constants.robotStates.State currentState = Constants.robotStates.State.FIRE_ON_THE_MOVE;
    private FieldConstants.FieldRegion currentRegion = FieldConstants.FieldRegion.UNKNOWN;
    public FieldConstants.FieldRegion getFieldLocation() { return currentRegion; }

    private double MaxSpeed = BoundaryConstants.MaxSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    private final CommandJoystick buttonBoard = new CommandJoystick(Constants.OperatorConstants.kButtonBoardPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final BoundaryManager boundaryManager = new BoundaryManager();

    private GenericEntry robotStateEntry;
    private GenericEntry robotFieldEntry;
    private GenericEntry robotPoseEntry;
    private GenericEntry turretAngleDisplay;
    private GenericEntry hoodAngleDisplay;

    private static final ShuffleboardTab robotStateTab = Shuffleboard.getTab("Robot State");

    public Trigger fireOnMoveTrigger() {
        return new Trigger(() -> currentState == Constants.robotStates.State.FIRE_ON_THE_MOVE);
    }

    public Trigger pushTrigger() {
        return new Trigger(() -> currentState == Constants.robotStates.State.PUSH);
    }

    public Trigger climbTrigger() {
        return new Trigger(() -> currentState == Constants.robotStates.State.CLIMB);
    }

    public Trigger idleTrigger() {
        return new Trigger(() -> currentState == Constants.robotStates.State.IDLE);
    }

    public Trigger stopAndShootTrigger() {
        return new Trigger(() -> currentState == Constants.robotStates.State.STOP_AND_SHOOT);
    }

    public Trigger isShootingTrigger() {
        return fireOnMoveTrigger().or(stopAndShootTrigger());
    }

    public Trigger unJamActive = buttonBoard.button(Constants.OperatorConstants.UNJAM_BUTTON);

    public Trigger fireButton() {
        return buttonBoard.button(Constants.OperatorConstants.FIRE_BUTTON);
    }

    public RobotContainer() {

        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        intakeExtender = new IntakeExtender();
        index = new IndexSubsystem();
        hood = new HoodSubsystem();
        turret = new TurretSubsystem();

        if (RobotBase.isSimulation()) {
            new SimulationExtensions(turret, hood);
            drivetrain.resetPose(new Pose2d( 0.5, 0.5, new Rotation2d(0)));
        }

        configureBindings();
        configureDefaults();
        configureShuffleboard();

        setRobotState(Constants.robotStates.State.FIRE_ON_THE_MOVE);  // temporarily moved from "IDLE" default to "FOM" default for testing purposes.
    }

    private void configureDefaults() {

        drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double leftY = joystick.getLeftY();
                double leftX = -joystick.getLeftX();
                double rightX = -joystick.getRightX();

                double deadband = 0.12;                             // added a deadband because my joysticks do not default to 0/0/0
                leftY = Math.abs(leftY) > deadband ? leftY : 0;
                leftX = Math.abs(leftX) > deadband ? leftX : 0;
                rightX = Math.abs(rightX) > deadband ? rightX : 0;

                double vx = leftX * MaxSpeed;
                double vy = leftY * MaxSpeed;
                double omega = rightX * MaxAngularRate;

                ChassisSpeeds raw = new ChassisSpeeds(vx, vy, omega);

                ChassisSpeeds safe = boundaryManager.constrainSpeeds(drivetrain.getPose(), raw);

                return drive.withVelocityX(safe.vxMetersPerSecond * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(safe.vyMetersPerSecond * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(safe.omegaRadiansPerSecond * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

        intake.setDefaultCommand(
            Commands.run(intake::stop, intake)
        ); // default to stopped when not called

        shooter.setDefaultCommand(
            Commands.run(shooter::stop, shooter)
        ); // default to stopped when not called

        intakeExtender.setDefaultCommand(
            Commands.run(intakeExtender::extenderUp, intakeExtender)
        ); // default to stopped when not called

        index.setDefaultCommand(
            Commands.run(index::stop, index)
        ); // default to up position when not called

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.



        // idleTrigger().negate().whileTrue(
        // // Drivetrain will execute this command periodically
        //     // drivetrain.applyRequest(() ->
        //     //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //     //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //     //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     // );
        //     drivetrain.applyRequest(() -> {
        //         double leftY = -joystick.getLeftY();
        //         double leftX = -joystick.getLeftX();
        //         double rightX = -joystick.getRightX();

        //         System.out.println("LeftY = " + leftY + "Left X = " + leftX + "Right X = " + rightX);

        //         double deadband = 0.1;
                
        //         leftY = Math.abs(leftY) > deadband ? leftY : 0;
        //         leftX = Math.abs(leftX) > deadband ? leftX : 0;
        //         rightX = Math.abs(rightX) > deadband ? rightX : 0;

        //         System.out.println("After deadband: Y=" + leftY + ", X=" + leftX + ", Rot=" + rightX);

        //         return drive
        //            .withVelocityX(leftY * MaxSpeed)
        //             .withVelocityY(leftX * MaxSpeed)
        //             .withRotationalRate(rightX * MaxAngularRate);
        //     })
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        joystick.b().whileTrue(new DynamicAimCommand(turret, hood, this));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().whileTrue(new IntakeForwardCommand(intake));

        buttonBoard.button(Constants.OperatorConstants.FIRE_ON_THE_MOVE_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.FIRE_ON_THE_MOVE)));
        buttonBoard.button(Constants.OperatorConstants.PUSH_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.PUSH)));
        buttonBoard.button(Constants.OperatorConstants.CLIMB_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.CLIMB)));
        buttonBoard.button(Constants.OperatorConstants.IDLE_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.IDLE)));
        buttonBoard.button(Constants.OperatorConstants.STOP_AND_SHOOT_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.STOP_AND_SHOOT)));

        isShootingTrigger().and(isShootingTrigger().negate().debounce(0.05)).onTrue( // This trigger will activate when we first enter the FIRE_ON_THE_MOVE state, but not on subsequent scheduler runs while we're still in that state, due to the debounce.
            new IntakeExtenderDown(intakeExtender)
        );

        isShootingTrigger().and(isShootingTrigger().negate().debounce(0.05)).onFalse( // This trigger will activate when we leave the FIRE_ON_THE_MOVE state, but not on subsequent scheduler runs while we're still outside that state, due to the debounce.
            new IntakeExtenderUp(intakeExtender)
        );

        // for fire on the  move, turn on intake, index, and shooter (unless we're unjamming)
        fireOnMoveTrigger().and(unJamActive.negate()).whileTrue(
            Commands.parallel(
                new ShooterContinuousCommand(shooter),
                new IndexToShooterCommand(index),
                new IntakeForwardCommand(intake)
            )
        );

        fireOnMoveTrigger().whileTrue(new DynamicAimCommand(turret, hood, this));

        // for stop and shoot, if we're not unjamming or shooting, turn on the intake, turn off shooter & index.
        stopAndShootTrigger()
            .and(unJamActive.negate())
            .and(fireButton().negate())
            .whileTrue(
            Commands.parallel(
                new ShooterContinuousCommand(shooter),
                new IndexStopCommand(index),
                new IntakeForwardCommand(intake)
            )
        );

        // for stop and shoot, if the fire button is being held, turn everything on
        stopAndShootTrigger()
            .and(fireButton())
            .and(unJamActive.negate())
            .whileTrue(
            Commands.parallel(
                new ShooterContinuousCommand(shooter),
                new IndexToShooterCommand(index),
                new IntakeForwardCommand(intake)
            )
        );

        // can't unjam while fire button is being pressed
        unJamActive.and(fireButton().negate()).whileTrue(
            Commands.repeatingSequence(
                new IndexReverseCommand(index).withTimeout(0.25),       // run index in reverse briefly to attempt to clear the jam
                new IndexToShooterCommand(index).withTimeout(0.25), // run index forward briefly to attempt to clear the jam after reversing
                Commands.waitSeconds(0.1)                          // wait 0.1 seconds between each cycle of unjamming to allow motors to respond and potentially clear the jam
            ).withTimeout(4.0)                                      // command will stop after 4 seconds even if the button is still held, to prevent potential damage from prolonged unjamming
        );

        pushTrigger().whileTrue(
            Commands.parallel(
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake)
            )
        );

        idleTrigger().whileTrue(
            Commands.parallel(
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake)
            )
        );

        climbTrigger().whileTrue(
            Commands.parallel(
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake)
                // Climb motors will be activated by separate triggers/buttons, not in this trigger
            )
        );
    }

    public void configureShuffleboard(){
        ShuffleboardControl.setupDashboard();

        // Display robot state - Fire on Move, Stop & Shoot, etc.
        robotStateEntry = robotStateTab
            .add("Robot State", "UNKNOWN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();


        // Display which region the robot is in - Blue Left, Red Right, etc.
        robotFieldEntry = robotStateTab
            .add("Field Location", "UNKNOWN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

        // Display robot coordinates - currently only Swerve odometry
        robotPoseEntry = robotStateTab
            .add("Pose X Y Angle", "0.00, 0.00, 0")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(3, 1)
            .getEntry();

        // display Hood angle for troubleshooting
        hoodAngleDisplay = robotStateTab
            .add("Hood Angle (deg)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", 90))
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();

        // display turret angle for troubleshooting
        turretAngleDisplay = robotStateTab
            .add("Turret Angle (deg)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -180, "max", 180))
            .withPosition(3, 3)
            .withSize(2, 1)
            .getEntry();


        ShuffleboardControl.registerOpenLoopMotor("Intake Roller", new MotorAccessor() {
            @Override public void setPower(double p) { intake.setPower(p); }
            @Override public double getPower() { return intake.getPower(); }
            @Override public void setSpeed(double rpm) {}
            @Override public double getSpeedRpm() { return 0.0; }
            @Override public void setPositionDegrees(double d) {}
            @Override public double getPositionDegrees() { return 0.0; }
            @Override public void applyPid(double kp, double ki, double kd) {}
        });

        ShuffleboardControl.registerVelocityMotor("Shooter", new MotorAccessor() {
            @Override public void setSpeed(double rpm) { shooter.setSpeed(rpm); }
            @Override public double getSpeedRpm() { return shooter.getSpeed(); }
            @Override public void setPower(double p) {}
            @Override public double getPower() { return 0.0; }
            @Override public void setPositionDegrees(double d) {}
            @Override public double getPositionDegrees() { return 0.0; }
            @Override public void applyPid(double kp, double ki, double kd) { shooter.applyPid(kp, ki, kd); }
        });

        ShuffleboardControl.registerPositionMotor("Intake Extender", new MotorAccessor() {
            @Override public void setPositionDegrees(double deg) { intakeExtender.setPositionDegrees(deg); }
            @Override public double getPositionDegrees() { return intakeExtender.getPositionDegrees(); }
            @Override public void setPower(double p) {}
            @Override public double getPower() { return 0.0; }
            @Override public void setSpeed(double rpm) {}
            @Override public double getSpeedRpm() { return 0.0; }
            @Override public void applyPid(double kp, double ki, double kd) { intakeExtender.applyPid(kp, ki, kd); }
        });

        ShuffleboardControl.registerPositionMotor("Hood", new MotorAccessor() {
            @Override public void setPositionDegrees(double deg) { hood.setPositionDegrees(deg); }
            @Override public double getPositionDegrees() { return hood.getPositionDegrees(); }
            @Override public void setPower(double p) {}
            @Override public double getPower() { return 0.0; }
            @Override public void setSpeed(double rpm) {}
            @Override public double getSpeedRpm() { return 0.0; }
            @Override public void applyPid(double kp, double ki, double kd) { hood.applyPid(kp, ki, kd); }
        });

        ShuffleboardControl.registerPositionMotor("Turret", new MotorAccessor() {
            @Override public void setPositionDegrees(double deg) { turret.setPositionDegrees(deg); }
            @Override public double getPositionDegrees() { return turret.getPositionDegrees(); }
            @Override public void setPower(double p) {}
            @Override public double getPower() { return 0.0; }
            @Override public void setSpeed(double rpm) {}
            @Override public double getSpeedRpm() { return 0.0; }
            @Override public void applyPid(double kp, double ki, double kd) { hood.applyPid(kp, ki, kd); }
        });

    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
        //I'm the bad guy!
    }

    private void setRobotState(Constants.robotStates.State newState) {
        currentState = newState;
        System.out.println("Robot state changed to: " + currentState);

        if (robotStateEntry != null){
            robotStateEntry.setString(getRobotStateAsString());
        }
        else {
            System.out.println("Warning: robotStateEntry is null - Shuffleboard not Initialized yet");
        }

        robotStateEntry.setString(getRobotStateAsString());
    }

    public String getRobotStateAsString() {
        return switch (currentState) {
            case FIRE_ON_THE_MOVE -> "Fire on the Move";
            case PUSH -> "Push";
            case CLIMB -> "Climb";
            case IDLE -> "Idle";
            case STOP_AND_SHOOT -> "Stop and Shoot";
            default -> "none";
        };
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }

    public ChassisSpeeds getRobotSpeeds(){
        return drivetrain.getCurrentRobotRelativeSpeeds();
    }

    public String getFieldRegionString() {
        Pose2d pose = getPose();
        FieldConstants.FieldRegion region = FieldConstants.CurrentFieldState(pose);
        return FieldConstants.toString(region);
    }

    public void updateFieldAndPoseDisplay(){
        // Get robot coordinates (pose)
        Pose2d pose = getPose();

        // Get region from robot coordinates
        currentRegion = FieldConstants.CurrentFieldState(pose);

        // update Shuffleboard while we're here
        if(robotFieldEntry != null) {
            robotFieldEntry.setString(FieldConstants.toString(currentRegion));
        }
        if(robotPoseEntry != null) {
            robotPoseEntry.setString(getPoseString());
        }
        if (hoodAngleDisplay != null) {
            hoodAngleDisplay.setDouble(hood.getPositionDegrees());
        }
        if(turretAngleDisplay != null) {
            turretAngleDisplay.setDouble(turret.getPositionDegrees());
        }
    }

    public String getPoseString() {
        Pose2d pose = getPose();
        return String.format("X: %.2f Y: %.2f Rot: %.1f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
}