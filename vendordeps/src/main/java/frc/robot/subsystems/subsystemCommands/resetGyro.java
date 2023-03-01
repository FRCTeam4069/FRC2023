package frc.robot.subsystems.subsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class resetGyro extends CommandBase{
    private final SwerveSubsystem swerve;

    public resetGyro(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    @Override
    public void execute(){
        swerve.resetGyro();
    }
    

}
