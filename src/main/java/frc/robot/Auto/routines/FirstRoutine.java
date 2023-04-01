package frc.robot.Auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HighPoseS1;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HighPoseS2;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.ShootCubeL3;
import frc.robot.Auto.Commands.drivebaseCommands.autoAlign;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;

public class FirstRoutine extends SequentialCommandGroup{
 
    public FirstRoutine(){
        addCommands(
        new ShootCubeL3() );       
        addCommands(new OpenIntake());
        addCommands(new 
        followTrajectoryCommand(PathPlanner.loadPath("red Short", 
        new PathConstraints(2, 2)), true)
        .alongWith(new wristToPosition(70, 2, 2, 2))
        );
        addCommands(new intakeToPose(2, .5, 0.5, 0));

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
