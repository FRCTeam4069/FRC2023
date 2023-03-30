package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Constants.IO.state;


public class MidPoseS1 extends SequentialCommandGroup {
    public MidPoseS1(){
        addCommands(new setLastState(state.MID));
        addCommands(
        (new fieldRelativeArmToPose(-62, (130-62))
        .andThen(new wristToPosition(120, 5, 0.5, 1)))
        .andThen(new extendToPose(4, 0.5))
        .andThen(new wristToPosition(25, 5, 0.25, .5))
        );
        

    }

    
}
