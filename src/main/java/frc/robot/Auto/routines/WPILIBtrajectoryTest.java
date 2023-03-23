package frc.robot.Auto.routines;

import java.io.IOError;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.swing.text.PlainDocument;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.subsystems.SwerveSubsystem;

public class WPILIBtrajectoryTest extends SequentialCommandGroup {
    private Trajectory trajectory = new Trajectory();    

    public WPILIBtrajectoryTest() {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("ONE");
        

        addCommands();
    }

}
