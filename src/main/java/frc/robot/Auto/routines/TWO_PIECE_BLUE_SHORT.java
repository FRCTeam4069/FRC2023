package frc.robot.Auto.routines;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.subsystems.swerveSubsystem;

public class TWO_PIECE_BLUE_SHORT extends SequentialCommandGroup {

    private final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TWO_PIECE_BLUE_SHORT",
            new PathConstraints(2, 2));
    private static final swerveSubsystem swerve = RobotContainer.swerveSubsystem;

    public TWO_PIECE_BLUE_SHORT() {
        Command cone1 = swerve.followTrajectoryCommand(pathGroup.get(0), true);
        Command placeCone1 = swerve.followTrajectoryCommand(pathGroup.get(1), true);
        Command spin2 = swerve.followTrajectoryCommand(pathGroup.get(2), true);
        Command end = swerve.followTrajectoryCommand(pathGroup.get(3), true);

        // place cube 
        addCommands((new armToPose(-60, false, 30)
                .alongWith(new wristToPosition(0, 5, 0.5, 1)))
                .andThen(new extendToPose(20, 0.5))
                .andThen(new OpenIntake()));

        // arm to other side, + home
        addCommands(new armToPose(20, false, 30).andThen(new HomePose()));
        addCommands(cone1.andThen());


    }

}
