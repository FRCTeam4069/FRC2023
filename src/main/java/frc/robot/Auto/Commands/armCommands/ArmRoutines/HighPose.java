package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO.state;

public class HighPose extends SequentialCommandGroup {

    public HighPose() {
        addCommands(new setLastState(state.HIGH));
        addCommands(
                (new armToPose(60, true, 30)
                        .alongWith(new wristToPosition(120, 5, 0.5, 1)))
                        .andThen(new extendToPose(23.5, 0.5))
                        .andThen(new wristToPosition(25, 5, 0.25, .5))
                        .andThen(new fieldRelativeArmToPose(70, 10))
                        .andThen(new wristToPosition(0, 5, 0.25, 0.5))
        );

    }
}
