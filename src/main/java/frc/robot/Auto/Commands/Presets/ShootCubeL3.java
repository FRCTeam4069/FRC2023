package frc.robot.Auto.Commands.Presets;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.timeBasedIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class ShootCubeL3 extends SequentialCommandGroup {

    public ShootCubeL3(){
        addCommands(
            new armToPose(-60, false, 10).alongWith(
        new wristToPosition(0, 2, 0.1, 0.5))
        .andThen(new timeBasedIntake(0,-0.2,0.2))
        .andThen(new armToPose(1, false, 10))
        );
    }

}

