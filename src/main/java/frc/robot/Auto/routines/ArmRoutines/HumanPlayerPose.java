package frc.robot.Auto.routines.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;

public class HumanPlayerPose extends SequentialCommandGroup{
    public HumanPlayerPose(){
        addCommands(
        new extendToPose(0, 1)
        .andThen(
        new moveToPose(40, true)
        .alongWith(new wristToPosition(-40, 5, 0.5, 2)))
        );
    }

}
