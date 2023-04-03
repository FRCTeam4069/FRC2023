package frc.robot.Auto.routines;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.armCommands.armToPose;
import frc.robot.Auto.Commands.armCommands.extendToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.subsystems.swerveSubsystem;

public class OVERSHOOT_RAMP extends SequentialCommandGroup {
    private final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("Overshoot ramp",
            new PathConstraints(1, 1.5));
    private static final swerveSubsystem swerve = RobotContainer.swerveSubsystem;
    private HashMap<String, Command> eventMap = new HashMap<>();

    public OVERSHOOT_RAMP() {
        FollowPathWithEvents command = new FollowPathWithEvents(
                swerve.followTrajectoryCommand(pathGroup, true),
                pathGroup.getMarkers(),
                eventMap);

        addCommands(
                (new armToPose(-60, false, 30)
                        .alongWith(new wristToPosition(0, 5, 0.5, 1)))
                        .andThen(new extendToPose(20, 0.5))
                        .andThen(new OpenIntake()));

        addCommands(
                new HomePose().alongWith(command).andThen(new WaitCommand(1)
                        .alongWith((new armToPose(-70, false, 15)
                                .andThen(new wristToPosition(-90, 10, 0.5, 1))
                                .andThen(new armToPose(130, false, 5)))))
                        );
    }

}
