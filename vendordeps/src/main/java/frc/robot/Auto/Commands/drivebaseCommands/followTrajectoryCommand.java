package frc.robot.Auto.Commands.drivebaseCommands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class followTrajectoryCommand extends CommandBase{
    private SwerveSubsystem swerve;
    private PathPlannerTrajectory traj;
    private boolean isFirstPath, pathFinished;


    public followTrajectoryCommand(SwerveSubsystem swerveI, PathPlannerTrajectory traj, boolean isFirstPath) {
        this.swerve = swerveI;
        this.isFirstPath = isFirstPath;
        this.traj = traj;
    }
    
    @Override
    public void execute() {
        Command path = swerve.followTrajectoryCommand(traj, isFirstPath);
        path.execute();
        pathFinished = path.isFinished();

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return pathFinished;
    }
}
