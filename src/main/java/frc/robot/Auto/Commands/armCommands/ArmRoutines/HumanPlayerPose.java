package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Constants.IO.state;

public class HumanPlayerPose extends SequentialCommandGroup {
    public HumanPlayerPose() {
        addCommands(new setLastState(state.HUMAN));
        addCommands(
        new extendToPose(0, 1)
                        .andThen(new fieldRelativeArmToPose(0, 25)
                            .andThen(new fieldRelativeArmToPose(28, 5)
                                .andThen(new wristToPosition(-100, 5, 0.5, 1)))));

    }

}
