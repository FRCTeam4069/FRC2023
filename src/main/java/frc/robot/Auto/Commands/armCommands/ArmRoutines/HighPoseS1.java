package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Constants.IO.state;

public class HighPoseS1 extends SequentialCommandGroup {

    public HighPoseS1() {
        addCommands(new setLastState(state.HIGH));
        addCommands(
                new fieldRelativeArmToPose(-60, 60)
                        .andThen(new wristToPosition(90, 5, 0.25, .5))
                        .andThen(new extendToPose(23.5, 0.5))
                        .andThen(new wristToPosition(0, 5, 0.5, 1))
                        
        );
    }
}
