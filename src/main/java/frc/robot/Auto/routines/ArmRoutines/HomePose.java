package frc.robot.Auto.routines.ArmRoutines;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;

public class HomePose extends SequentialCommandGroup{
    public HomePose(){
        addCommands(
        new extendToPose(0, 1)
        .andThen(
        new moveToPose(130, true)
        .alongWith(new wristToPosition(120, 10, 0.5, 2)))
        .andThen(new extendToPose(0, 1))
        );
    }

}
