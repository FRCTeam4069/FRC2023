package frc.robot.Auto.routines.RoutinesV2;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Commands.Presets.AUTOgroundConeS1;
import frc.robot.Auto.Commands.Presets.AUTOgroundConeS2;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.Presets.ShootCubeL3;
import frc.robot.Auto.Commands.Presets.placeCubeL3;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoAlign;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.drivebaseCommands.coneAlign;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.scalableWristToPosition;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class Middle_OneAndTaxi extends SequentialCommandGroup {
    public PathPlannerTrajectory path0 = PathPlanner.loadPath("M_Taxi",
            new PathConstraints(1.5, 2));

    public Middle_OneAndTaxi() {
        addCommands(
                new ShootCubeL3()
                .andThen(new armToPose(-5, false, 5))
                .andThen(new HomePose())
                .andThen( new followTrajectoryCommand(path0, true) )
                .andThen(new autoBalance(10, false)));
    }

}
