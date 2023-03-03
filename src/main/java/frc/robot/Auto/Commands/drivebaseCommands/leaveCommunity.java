package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class leaveCommunity extends CommandBase{
    private static final SwerveSubsystem driveSubsystem = RobotContainer.swerveSubsystem;
    public double poseX, speed;


    public leaveCommunity() {
       addRequirements(RobotContainer.swerveSubsystem); 
    }
    
    @Override
    public void execute() {
        driveSubsystem.resetOdometry();
        poseX = driveSubsystem.odometry.getPoseMeters().getX();
        speed = MathUtil.clamp(((-165 - poseX)*0.1), -2,0);
        driveSubsystem.setModuleStates(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(speed, 0, 0)));

        //get to x = -163
    }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return (poseX > 165);
    }
}
