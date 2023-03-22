package frc.robot.Auto.routines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPathPlannerPath extends SequentialCommandGroup {
    private final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("Two Items", new PathConstraints(4, 3));
    private static final SwerveSubsystem swerve = RobotContainer.swerveSubsystem;

    private HashMap<String, Command> eventMap = new HashMap<>();
    

    public TestPathPlannerPath() {
        eventMap.put("CloseIntake", new timeBasedIntake(0.5, 1));
        FollowPathWithEvents command = new FollowPathWithEvents(
                swerve.followTrajectoryCommand(pathGroup, true),
                pathGroup.getMarkers(),
                eventMap);

        addCommands(command);
    }

}
