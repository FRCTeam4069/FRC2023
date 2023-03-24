package frc.robot.Auto.Commands.armCommands.ArmRoutines;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO;
import frc.robot.Constants.IO.state;

public class HomePose extends SequentialCommandGroup{
    public HomePose(){
        addCommands(new setLastState(state.HOME));
        addCommands(
        new extendToPose(0, 0.5)
        .alongWith(new wristToPosition(120, 10, 0.5, 1))
        .andThen(new armToPose(130, true, 5))
        .andThen(new extendToPose(0, 1))
        );

        // addCommands(
        //     new extendToPose(0, 2)
        //     .alongWith(new wristToPosition(120, 10, 0.5, 1))
        //     .andThen(new armToPose(130, true, 5)            
        //     )            .andThen(new extendToPose(0, 0.5))

        //     );
    
        
    }

}
