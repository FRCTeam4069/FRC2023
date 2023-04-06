package frc.robot.Auto.routines.RoutinesV2.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.Presets.ShootCubeL3;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoAlignAuto;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.scalableWristToPosition;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class Shortside_Two extends SequentialCommandGroup {
    public PathPlannerTrajectory path0 = PathPlanner.loadPath("B_Short_s0",
            new PathConstraints(5, 4));
    public PathPlannerTrajectory path1 = PathPlanner.loadPath("B_Short_s1",
            new PathConstraints(5, 4));

            
    public Shortside_Two() {

        addCommands(
                new ShootCubeL3());
        addCommands(new followTrajectoryCommand(path0, true).alongWith(new HomePose()).alongWith(new OpenIntake()));
        addCommands(new intakeToPose(1, .5, 0.5, .5)
                .andThen(
                    new extendToPose(0, 0.5)
                    .andThen(new armToPose(130, true, 5).alongWith(new wristToPosition(75, 10, 0.5, 1)))
                    .andThen(new extendToPose(0, 1))
                ));

        addCommands(new followTrajectoryCommand(path1, false)
                .alongWith(new wristToPosition(100, 2, 1, 1))
                .alongWith(new intakeToPose(1, .2, 0.5, 0)));

        addCommands(
                new autoAlignAuto(1.5)
                        .alongWith(

                                (new fieldRelativeArmToPose(-55, 60)
                                        .alongWith(new WaitCommand(0.2)
                                                .alongWith(new scalableWristToPosition(90, 5, 0.25, .5, 0.5)))
                                        .andThen(new extendToPose(23.5, 0.5))
                                        .andThen(new wristToPosition(0, 5, 0.2, 1))

                                ).andThen(

                                        new armToPose(70, true, 5)
                                                .andThen(new wristToPosition(0, 5, 0.1, 0.5))
                                                .andThen(new OpenIntake())
                                                .andThen(new armToPose(60, true, 5))
                                                .andThen(new HomePose())

                                ))

        );
    }

}
