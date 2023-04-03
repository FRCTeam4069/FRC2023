package frc.robot.Auto.Commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class AUTOgourdCone extends SequentialCommandGroup {
    public AUTOgourdCone(){
        
        addCommands(
            new extendToPose(0, 0.5)
            .andThen(new fieldRelativeArmToPose(100, 5)
            .alongWith(new wristToPosition(-50, 10, 0.5, 1)))
            .andThen(new extendToPose(0, 1))


        );
    }    
}
