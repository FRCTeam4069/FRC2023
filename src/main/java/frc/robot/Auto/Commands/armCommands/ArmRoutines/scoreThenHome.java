package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Constants.IO.state;

public class scoreThenHome extends SequentialCommandGroup {
    public scoreThenHome() {

        addCommands(
                (new OpenIntake())
                        .andThen((new armToPose(60, true, 5)))
                        .andThen(new extendToPose(0, 1))
                        .andThen(
                                new armToPose(130, true, 5)
                                .alongWith(new wristToPosition(120, 10, 0.5, 1)))
                        .andThen(new extendToPose(0, 1))

        );
        addCommands(new setLastState(state.HOME));

    }

}
