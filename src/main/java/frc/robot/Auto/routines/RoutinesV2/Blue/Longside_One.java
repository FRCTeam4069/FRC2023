package frc.robot.Auto.routines.RoutinesV2.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.Presets.placeCubeL3;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;

public class Longside_One extends SequentialCommandGroup {
    private final PathPlannerTrajectory path = PathPlanner.loadPath("long_B",
            new PathConstraints(4, 2));

    public Longside_One() {

        addCommands(
                new placeCubeL3()
                .andThen(  new followTrajectoryCommand(path, true) )
        );
    }

}
