package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.BallisticCalculator;
import frc.robot.BallisticCalculator.BallisticSolution;

public class StaticAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem hood;
    private final TurretSubsystem turret;           // included for completeness, but not moved
    private final BallisticCalculator calculator;

    // PID for robot rotation (tune these!)
    private final PIDController rotationPid = new PIDController(
        5.0,   // kP - quick response
        0.0,   // kI - usually 0 for heading
        0.8    // kD - damp oscillation
    );

    private Rotation2d desiredHeading;
    private boolean rotationFinished = false;
    private boolean hoodFinished = false;

    public StaticAimCommand(
        CommandSwerveDrivetrain drivetrain,
        HoodSubsystem hood,
        TurretSubsystem turret,
        BallisticCalculator calculator
    ) {
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.turret = turret;
        this.calculator = calculator;

        addRequirements(drivetrain, hood, turret);

        // Configure PID
        rotationPid.setTolerance(Math.toRadians(3.0));
        rotationPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Reset PID
        rotationPid.reset();

        // Get current target position
        Pose2d targetPose = getPrimaryTargetPosition();
        if (targetPose == null) {
            end(true);  // no target, cancel
            return;
        }

        Pose2d robotPose = drivetrain.getPose();

        // Desired robot heading: face the target
        desiredHeading = targetPose.minus(robotPose.getTranslation()).getAngle();

        // Lock turret to center (0° relative to robot back)
        turret.setPositionDegrees(0.0);

        // Hood will be updated in execute
        hoodFinished = false;
        rotationFinished = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Pose2d targetPose = getPrimaryTargetPosition();

        if (targetPose == null) return;

        // 1. Robot rotation
        double currentHeadingRad = robotPose.getRotation().getRadians();
        double errorRad = desiredHeading.getRadians() - currentHeadingRad;
        errorRad = (errorRad + Math.PI) % (2 * Math.PI) - Math.PI;  // shortest path

        double rotationalRate = rotationPid.calculate(errorRad);

        drivetrain.applyRequest(() -> 
            new SwerveRequest.FieldCentric()
                .withVelocityX(0.0)  // no translation during aim
                .withVelocityY(0.0)
                .withRotationalRate(rotationalRate)
        );

        rotationFinished = rotationPid.atSetpoint();

        // 2. Hood adjustment (ballistic calc)
        BallisticSolution solution = calculator.getAngles(
            robotPose,
            targetPose.getTranslation(),
            drivetrain.getCurrentRobotRelativeSpeeds(),
            Constants.ballisticConstants.HUB_HEIGHT  // or dynamic height
        );

        if (solution.isValid()) {
            hood.setPositionDegrees(solution.hoodAngle);
            hoodFinished = hood.atSetpoint();  // your hood tolerance method
        } else {
            hoodFinished = false;
        }
    }

    @Override
    public boolean isFinished() {
        return rotationFinished && hoodFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot rotation
        drivetrain.applyRequest(() -> new SwerveRequest.Idle());

        // Hood stays at last setpoint (active hold)
        // Turret stays centered

        if (interrupted) {
            System.out.println("StaticAimCommand interrupted");
        }
    }
}