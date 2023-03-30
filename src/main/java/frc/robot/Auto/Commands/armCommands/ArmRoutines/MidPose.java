package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Constants.IO.state;


public class MidPose extends SequentialCommandGroup {
    public MidPose(){
        addCommands(new setLastState(state.MID));
        // addCommands(
        // (new armToPose(63, true, 30)
        // .alongWith(new wristToPosition(120, 5, 0.25, .5)))
        // .andThen(new extendToPose(2, 0.5)).andThen(new wristToPosition(30, 5,0.25, .5))
        // );


        addCommands(
        (new armToPose(62, true, 30)
        .alongWith(new wristToPosition(120, 5, 0.5, 1)))
        .andThen(new extendToPose(2, 0.5))
        .andThen(new wristToPosition(25, 5, 0.25, .5))
        .andThen(new armToPose(73, true, 10))
        .andThen(new wristToPosition(0, 5, 0.25, 0.5))
        );
        

    }

    
}
