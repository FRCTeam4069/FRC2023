package frc.robot.Auto.Commands.drivebaseCommands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class rotate extends CommandBase {
    private final SwerveSubsystem swerve = RobotContainer.swerveSubsystem;
    private final double targetHeading, Timeout;
    private final Timer timer = new Timer();
    public double currAngle;
    private final PIDController pidController = new PIDController(0.05, 0, 0);

    public rotate(double angle, double Timeout) {
        this.targetHeading = angle;
        this.Timeout = Timeout;
        pidController.enableContinuousInput(-180, 180);
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
        pidController.setSetpoint(targetHeading);

    }

    @Override
    public void execute() {
        currAngle = swerve.odometry.getPoseMeters().getRotation().getDegrees();
        ChassisSpeeds turnChassisSpeeds;
        swerve.setModuleState(kinematics.m_kinematics
                .toSwerveModuleStates(
                    new ChassisSpeeds(0,
                     0,
                     pidController.calculate(targetHeading))));

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Timeout);
    }

}
