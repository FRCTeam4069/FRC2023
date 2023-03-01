package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class moveToPose extends CommandBase{
    private double targetPose;
    private int ElapsedTime = 0,_Timeout = 100;
    private boolean errorOccured;
    public moveToPose(double position){

        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute(){
        if( RobotContainer.arm.moveToPos(targetPose, 1) ){

        }else errorOccured = true;
        
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
    
}
