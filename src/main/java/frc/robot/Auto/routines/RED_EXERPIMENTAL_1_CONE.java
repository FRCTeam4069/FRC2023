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
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.intakeAndWristCommands.timeBasedIntake;
import frc.robot.subsystems.swerveSubsystem;

public class RED_EXERPIMENTAL_1_CONE extends SequentialCommandGroup {
    private final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("RED_SHORT_1_CUBE",
            new PathConstraints(2, 2));
    private static final swerveSubsystem swerve = RobotContainer.swerveSubsystem;
    private HashMap<String, Command> eventMap = new HashMap<>();

    public RED_EXERPIMENTAL_1_CONE() {
        /* 
        Command goToP1 = swerve.followTrajectoryCommand(null, isFinished())
        Command goToP2 = 
        Command goToP3 = 
        Command goToP4 = 
        
        addCommands(
        new placeCubeL3().andThen(command));
        */
    }

}
