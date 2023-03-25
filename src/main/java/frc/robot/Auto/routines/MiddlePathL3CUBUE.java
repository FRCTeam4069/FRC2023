package frc.robot.Auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class MiddlePathL3CUBUE extends SequentialCommandGroup {

    public MiddlePathL3CUBUE() {
        addCommands(new placeCubeL3().andThen(new autoBalance(10, false)));

    }

}