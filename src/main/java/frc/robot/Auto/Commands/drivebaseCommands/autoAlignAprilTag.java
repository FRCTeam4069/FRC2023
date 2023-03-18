package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class autoAlignAprilTag extends CommandBase {
    private final double xOffset;
    private final SwerveSubsystem m_drivebase = RobotContainer.swerveSubsystem;
    private final double yOffset;

    public autoAlignAprilTag(double xOffset, double yOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        addRequirements(RobotContainer.swerveSubsystem);
    }
    
    @Override
    public void execute(){


    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
