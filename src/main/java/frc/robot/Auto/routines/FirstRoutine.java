package frc.robot.Auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.Presets.AUTOgourdCone;
import frc.robot.Auto.Commands.Presets.HighPoseS1;
import frc.robot.Auto.Commands.Presets.HighPoseS2;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.Presets.ShootCubeL3;
import frc.robot.Auto.Commands.drivebaseCommands.autoAlign;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class FirstRoutine extends SequentialCommandGroup{
 
    public FirstRoutine(){
        addCommands(
        new ShootCubeL3().andThen(new AUTOgourdCone().alongWith(new OpenIntake())) );       
        addCommands(new 
        followTrajectoryCommand(PathPlanner.loadPath("BLUE SHORT", 
        new PathConstraints(4, 2)), true));
        addCommands(new intakeToPose(1, .5, 0.5, 0).andThen(
            new HomePose()));

        addCommands(new 
        followTrajectoryCommand(PathPlanner.loadPath("RS_C1", 
        new PathConstraints(2, 2)), false)
        .alongWith(new wristToPosition(100, 2, 1, 1))
        );

        addCommands(
            new autoAlign(2).andThen(new HighPoseS1()).andThen(new HighPoseS2())
        );
    }

}
