package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class followTrajectoryCommand extends CommandBase {
    private static final SwerveSubsystem driveSubsystem = RobotContainer.swerveSubsystem;
    private final Pose2d targetPose2d;
    private final boolean firstPath;

    public followTrajectoryCommand(Pose2d targetPose, boolean firstPath) {
        this.targetPose2d = targetPose;
        this.firstPath = firstPath;
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
