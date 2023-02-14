package frc.robot.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class followTrajectoryCommand extends CommandBase{
    private SwerveSubsystem swerve;


    public followTrajectoryCommand(SwerveSubsystem swerveI, PathPlannerTrajectory traj, boolean isFirstPath) {
        this.swerve = swerveI;
        SequentialCommandGroup Run = new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                  swerve.resetOdometry(traj.getInitialPose());
              }
            }),
            new PPSwerveControllerCommand(
            traj, 
            swerve::getPose, // Pose supplier
            DrivebaseConstants.m_kinematics, // SwerveDriveKinematics
            new PIDController(.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerve::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // Requires this drive subsystem
        )
        );
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
