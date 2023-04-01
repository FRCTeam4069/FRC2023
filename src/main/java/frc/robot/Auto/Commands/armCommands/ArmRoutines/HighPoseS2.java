package frc.robot.Auto.Commands.armCommands.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Constants.IO.state;

public class HighPoseS2 extends SequentialCommandGroup {

    public HighPoseS2() {
        addCommands(new setLastState(state.HIGH));
        addCommands(
                new armToPose(70, true, 5)
                        .andThen(new wristToPosition(0, 5, 0.25, 0.5))
                        .andThen(new OpenIntake())
                        .andThen(new armToPose(60, true, 5))
                        .andThen(new HomePose())
                        );


    }
}
