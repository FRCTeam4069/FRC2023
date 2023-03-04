package frc.robot.Auto.Commands.drivebaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class leaveCommunity extends CommandBase{
    private static final SwerveSubsystem driveSubsystem = RobotContainer.swerveSubsystem;
    private double speed, targetPose;
    private Supplier<Double> poseX;

    public leaveCommunity(double targetPose, Supplier<Double> posex) {
        this.targetPose = targetPose;
        this.poseX = posex;
        driveSubsystem.resetOdometry();
       addRequirements(RobotContainer.swerveSubsystem); 
    }
    
    @Override
    public void execute() {
        speed = MathUtil.clamp(((targetPose - poseX.get())*0.1), -2,0);
        driveSubsystem.setModuleStates(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(speed, 0, 0)));
        SmartDashboard.putNumber("PoseX",poseX.get());
        SmartDashboard.putNumber("TargetPose", targetPose);

        //get to x = -163
    }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(targetPose) < Math.abs(poseX.get()));
    }
}
