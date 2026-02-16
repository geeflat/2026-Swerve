package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BallisticCalculator;
import frc.robot.Constants;
import frc.robot.FieldConstants;           // for hub positions
import frc.robot.RobotContainer;          // to get current field region / pose

public class DynamicAimCommand extends Command {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final RobotContainer robotContainer;   // for pose, speeds, and field state

    public DynamicAimCommand(TurretSubsystem turret, HoodSubsystem hood, RobotContainer robotContainer) {
        this.turret = turret;
        this.hood = hood;
        this.robotContainer = robotContainer;

        addRequirements(turret, hood);   // important: prevents conflicts
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotContainer.getPose();
        ChassisSpeeds robotSpeeds = robotContainer.getRobotSpeeds();

        // Debug message
        System.out.println("Pose X = "+ robotPose.getX() + " Y = " + robotPose.getY() + " T = " + robotPose.getRotation());

        // Get the correct hub target based on current field region / alliance
        Translation2d targetPos = getTargetHubPosition();

        BallisticCalculator.BallisticSolution solution = 
            BallisticCalculator.getAngles(
                robotPose,
                targetPos,
                robotSpeeds,
                Constants.ballisticConstants.HUB_HEIGHT
            );

        turret.setPositionDegrees(solution.turretAngleDegrees);
        hood.setPositionDegrees(solution.hoodAngleDegrees);
    }

    private Translation2d getTargetHubPosition() {
        // Use your existing field state logic
        FieldConstants.FieldRegion region = robotContainer.getFieldLocation();

        if (region.toString().contains("BLUE")) {
            return FieldConstants.BLUE_HUB_TARGET;   // define these in FieldConstants
        } else if (region.toString().contains("RED"))
        {
            return FieldConstants.RED_HUB_TARGET;
        }
        return FieldConstants.RED_HUB_TARGET;
    }

    @Override
    public boolean isFinished() {
        return false;   // runs until interrupted
    }
}