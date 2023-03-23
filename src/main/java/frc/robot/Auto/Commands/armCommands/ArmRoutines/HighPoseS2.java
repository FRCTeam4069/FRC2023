package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO;
import frc.robot.Constants.IO.state;

public class HighPoseS2 extends SequentialCommandGroup {

    public HighPoseS2() {
        addCommands(new setLastState(state.HIGH));
        addCommands(
                new armToPose(70, true, 10)
                        .andThen(new wristToPosition(0, 5, 0.25, 0.5))
                        .andThen(new OpenIntake())
                        .andThen(new armToPose(60, true, 5))
                        .andThen(new HomePose())
                        );


    }
}
