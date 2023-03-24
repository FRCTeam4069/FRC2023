package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO;
import frc.robot.Constants.IO.state;


public class MidPoseS1 extends SequentialCommandGroup {
    public MidPoseS1(){
        addCommands(new setLastState(state.MID));
        // addCommands(
        // (new armToPose(63, true, 30)
        // .alongWith(new wristToPosition(120, 5, 0.25, .5)))
        // .andThen(new extendToPose(2, 0.5)).andThen(new wristToPosition(30, 5,0.25, .5))
        // );


        addCommands(
        (new armToPose(62, true, 30)
        .alongWith(new wristToPosition(120, 5, 0.5, 1)))
        .andThen(new extendToPose(4, 0.5))
        .andThen(new wristToPosition(25, 5, 0.25, .5))
        );
        

    }

    
}
