package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class followTrajectoryCommand extends CommandBase{
    private static final SwerveSubsystem driveSubsystem = RobotContainer.swerveSubsystem;

    public followTrajectoryCommand() {
       

      
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
