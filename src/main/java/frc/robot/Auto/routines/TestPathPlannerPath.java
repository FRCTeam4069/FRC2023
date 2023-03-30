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
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.intakeAndWristCommands.timeBasedIntake;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPathPlannerPath extends SequentialCommandGroup {
    // /private final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("ONE",
    // new PathConstraints(2, 2));
    private final PathPlannerTrajectory Path1, Path2, Path3, Path4, Path5;

    private static final SwerveSubsystem swerve = RobotContainer.swerveSubsystem;


    public TestPathPlannerPath() {
        // eventMap.put("CloseIntake", new timeBasedIntake(0.5, 1));
        // eventMap.put("CUBE_L3", new placeCubeL3());
        // eventMap.put("ARM_DOWN", new HomePose());

        Path1 = PathPlanner.generatePath(
                new PathConstraints(3, 2),
                new PathPoint(new Translation2d(2,4.38), Rotation2d.fromDegrees(27), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(6.52 , 4.6), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));
        Path2 = PathPlanner.generatePath(
                new PathConstraints(2, 2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)));
        Path3 = PathPlanner.generatePath(
                new PathConstraints(2, 2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)));
        Path4 = PathPlanner.generatePath(
                new PathConstraints(2, 2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)));
        Path5 = PathPlanner.generatePath(
                new PathConstraints(2, 2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)));

                Command CPath1 = swerve.followTrajectoryCommand(Path1, true);
                Command CPath2 = swerve.followTrajectoryCommand(Path2, false);
                Command CPath3 = swerve.followTrajectoryCommand(Path3, false);
                Command CPath4 = swerve.followTrajectoryCommand(Path4, false);
                Command CPath5 = swerve.followTrajectoryCommand(Path5, false);

        addCommands(CPath1);
    }

}
