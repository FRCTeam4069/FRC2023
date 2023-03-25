package frc.robot.Auto.routines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.subsystems.SwerveSubsystem;

public class OVERSHOOT_RAMP extends SequentialCommandGroup {
    private final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("Overshoot ramp",
            new PathConstraints(1, 2));
    private static final SwerveSubsystem swerve = RobotContainer.swerveSubsystem;
    private HashMap<String, Command> eventMap = new HashMap<>();

    public OVERSHOOT_RAMP() {
        FollowPathWithEvents command = new FollowPathWithEvents(
                swerve.followTrajectoryCommand(pathGroup, true),
                pathGroup.getMarkers(),
                eventMap);

        addCommands(
        new placeCubeL3().andThen(command)
        .andThen(new WaitCommand(1)
        .alongWith(new armToPose(90, false, 10)))
        .andThen(new autoBalance(100, true)));
    }

}
