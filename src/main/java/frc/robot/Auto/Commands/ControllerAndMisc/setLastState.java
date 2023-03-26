package frc.robot.Auto.Commands.ControllerAndMisc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IO;

public class setLastState extends CommandBase{
    private final IO.state nextState;
    public setLastState(IO.state state){
        this.nextState = state;

    }

    @Override
    public void execute() {
        IO.LastState = nextState;
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
