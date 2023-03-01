package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.armAndIntakeConstants;
import frc.robot.subsystems.armSubsystem;

public class extendToPose extends CommandBase{
    private double newpose;
    private final armSubsystem arm;
    public extendToPose(armSubsystem arm, double pose){
        this.arm = arm; 
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
