package frc.robot.Auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class Middle_Path_0cones extends SequentialCommandGroup {

    public Middle_Path_0cones() {
        addCommands(new moveToPose(-130, 1).alongWith(new autoBalance()));

    }

}