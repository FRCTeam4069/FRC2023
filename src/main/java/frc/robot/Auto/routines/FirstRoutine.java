package frc.robot.Auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Commands.Presets.AUTOgroundConeS1;
import frc.robot.Auto.Commands.Presets.AUTOgroundConeS2;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.Presets.ShootCubeL3;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.armCommands.fieldRelativeArmToPose;
import frc.robot.Auto.Commands.drivebaseCommands.autoAlign;
import frc.robot.Auto.Commands.drivebaseCommands.coneAlign;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.scalableWristToPosition;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class FirstRoutine extends SequentialCommandGroup {
        public PathPlannerTrajectory path0 = PathPlanner.loadPath("BLUE SHORT",
        new PathConstraints(5, 4));
        public PathPlannerTrajectory path1 = PathPlanner.loadPath("RS_C1",
        new PathConstraints(5, 4));

    public FirstRoutine() {
        addCommands(
                new ShootCubeL3());
        addCommands(new followTrajectoryCommand(path0, true)
                .alongWith(new AUTOgroundConeS1().alongWith(new OpenIntake()))
                .andThen(new coneAlign(1)));

        addCommands(new AUTOgroundConeS2()
                .andThen(new intakeToPose(1, .5, 0.5, 0))
                .andThen(new HomePose()));

        addCommands(new followTrajectoryCommand(path1, false)
                .alongWith(new wristToPosition(100, 2, 1, 1)));

        // auto alignment and placing sequence
        addCommands(
                new autoAlign(1.5)
                        .alongWith(new fieldRelativeArmToPose(-55, 60)
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
                        



                        ));
    }

}
