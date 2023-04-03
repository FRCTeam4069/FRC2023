package frc.robot.Auto.routines;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerveSubsystem;

public class TwoThings extends SequentialCommandGroup {
    private final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("PathGroup",
            new PathConstraints(2, 2));
    private static final swerveSubsystem swerve = RobotContainer.swerveSubsystem;

    public TwoThings() {

        addCommands(swerve.followTrajectoryCommand(pathGroup, true));
    }

}
