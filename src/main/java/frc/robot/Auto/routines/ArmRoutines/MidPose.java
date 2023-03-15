package frc.robot.Auto.routines.ArmRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;


public class MidPose extends SequentialCommandGroup {
    public MidPose(){
        addCommands(
        (new moveToPose(60, true)
        .alongWith(new wristToPosition(0, 5, 0.5, 2)))
        .andThen(new extendToPose(4.5, 0.5)));
    }

    
}
