package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.IO.state;

public class MidPoseS2 extends SequentialCommandGroup {
    public MidPoseS2() {
        addCommands(new setLastState(state.MID));
        // addCommands(
        // (new armToPose(63, true, 30)
        // .alongWith(new wristToPosition(120, 5, 0.25, .5)))
        // .andThen(new extendToPose(2, 0.5)).andThen(new wristToPosition(30, 5,0.25,
        // .5))
        // );

        addCommands(
                new armToPose(73, true, 10)
                        .andThen(new wristToPosition(0, 5, 0.25, 0.5))
                        .andThen(new OpenIntake())
                        .andThen(new armToPose(60, true, 7))
                        .andThen(new HomePose())

        );

    }

}
