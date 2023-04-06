package frc.robot.Auto.routines.RoutinesV2.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.Presets.placeCubeL3;
import frc.robot.Auto.Commands.drivebaseCommands.AutoCommands.followTrajectoryCommand;

public class Shortside_One extends SequentialCommandGroup {
    private final PathPlannerTrajectory path = PathPlanner.loadPath("short_R",
            new PathConstraints(4, 2));

    public Shortside_One() {
        addCommands(
                new placeCubeL3()
                .andThen(  new followTrajectoryCommand(path, true) )
        );
    }

}
