package frc.robot.Auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class Middle_Path_0cones extends SequentialCommandGroup {

    public Middle_Path_0cones() {
        addCommands(new armToPose(-130, false, 5));

    }

}