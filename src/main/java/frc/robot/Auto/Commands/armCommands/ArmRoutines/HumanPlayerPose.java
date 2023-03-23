package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO.state;

public class HumanPlayerPose extends SequentialCommandGroup {
    public HumanPlayerPose() {
        addCommands(new setLastState(state.HUMAN));
        addCommands(
        new extendToPose(0, 1)
                        .andThen(new armToPose(40, true, 5)
                                .andThen(new wristToPosition(-40, 5, 0.5, 1))));

    }

}
