package frc.robot.Auto.routines.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;

public class HighPose extends SequentialCommandGroup {

    public HighPose(){
        addCommands(
        (new moveToPose(60, true)
        .alongWith(new wristToPosition(0, 5, 0.5, 1)))
        .andThen(new extendToPose(23.5, 0.5)));
    }
}
