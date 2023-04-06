package frc.robot.Auto.Commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class AUTOgroundConeS2 extends SequentialCommandGroup {
    public AUTOgroundConeS2(){
        
        addCommands(
            new fieldRelativeArmToPose(110, 5)
            .alongWith(new wristToPosition(-30, 10, 0.5, 1))


        );
    }    
}
