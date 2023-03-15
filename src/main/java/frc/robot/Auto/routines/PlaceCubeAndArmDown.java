package frc.robot.Auto.routines;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.intakeCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Constants.armAndIntakeConstants.armConstants;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class PlaceCubeAndArmDown extends SequentialCommandGroup {

    public PlaceCubeAndArmDown() {
        addCommands(new moveToPose(-55, false)
        .alongWith(new wristToPosition(0, 5 , 0.5, 3))
        .andThen(new OpenIntake())
        .andThen(new wristToPosition(-1, 5, 0.5, 3)
        .andThen(new moveToPose(-130, false)).alongWith(new autoBalance())));
    }

}