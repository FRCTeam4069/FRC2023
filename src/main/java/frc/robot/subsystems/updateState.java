package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States;
import frc.robot.Constants.States.armState;
import frc.robot.Constants.States.intakeState;
import frc.robot.Constants.States.robotState;

public class updateState extends SubsystemBase {
    public static Enum[] states;


    @Override
    public void periodic(){

    }

    public void updateState(){
        if(States.currArmState.equals(armState.ABOVE_POLES)){

        }

    }

    


    
}
