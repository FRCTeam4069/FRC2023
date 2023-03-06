package frc.robot.Auto.routines;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.intakeCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class placeCube extends SequentialCommandGroup {

    public placeCube() {
        addCommands(new moveToPose(-55, 1)
        .andThen(new wristToPosition(0 ))
        .andThen(new OpenIntake()));

    }

}