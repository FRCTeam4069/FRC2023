package frc.robot.Auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.Commands.ControllerAndMisc.setLastState;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Constants.IO.state;

public class placeCubeL3 extends SequentialCommandGroup {

    public placeCubeL3() {
        addCommands(new setLastState(state.HIGH));
        addCommands(
                (new armToPose(-60, false, 30)
                        .alongWith(new wristToPosition(0, 5, 0.5, 1)))
                        .andThen(new extendToPose(20, 0.5))
                        .andThen(new OpenIntake())
                        .andThen(new HomePose()));

    }
}
