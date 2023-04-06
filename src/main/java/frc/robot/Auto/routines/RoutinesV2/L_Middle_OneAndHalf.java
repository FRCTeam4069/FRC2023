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

public class L_Middle_OneAndHalf extends SequentialCommandGroup {
    public PathPlannerTrajectory path0 = PathPlanner.loadPath("M_Left",
            new PathConstraints(5, 4));

    public L_Middle_OneAndHalf() {
        addCommands(
                new placeCubeL3());

        addCommands(new followTrajectoryCommand(path0, true)
        .alongWith(new AUTOgroundConeS1().alongWith(new OpenIntake()))
        .andThen(new coneAlign(1)));

        addCommands(new AUTOgroundConeS2()
        .andThen(new intakeToPose(1, .5, 0.5, 0))
        .andThen(new HomePose())
        .andThen(new autoBalance(6, true)
        ));

        // addCommands(new followTrajectoryCommand(path1, false)
        // .alongWith(new wristToPosition(100, 2, 1, 1)));

    }

}
