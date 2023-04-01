package frc.robot.Auto.Commands.drivebaseCommands;

import java.util.function.Supplier;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.LimeLight1;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.limeLight;

public class autoAlign extends CommandBase {
    private final Timer timer = new Timer();
    private final double timeToAlign;
    private final SwerveSubsystem swerveSubsystem = RobotContainer.swerveSubsystem;
    private final limeLight f_LimeLight = RobotContainer.frontLimeLight;
    private final limeLight b_LimeLight = RobotContainer.backLimeLight;
    private limeLight cameraToUse;

    private PIDController turninPID = new PIDController(0.07, 0, 0);

    public autoAlign(double timeToAlign) {

        turninPID.enableContinuousInput(-180, 180);
        this.timeToAlign = timeToAlign;

        addRequirements(RobotContainer.swerveSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.stop();
    }

    @Override
    public void execute() {
        timer.start();
        updateUsedCamera();
        f_LimeLight.setMode(0);
        b_LimeLight.setMode(0);
        f_LimeLight.setLED(3);
        b_LimeLight.setLED(3);
        b_LimeLight.setPipeline(0);
        f_LimeLight.setPipeline(0);
        ChassisSpeeds chassisSpeeds;
        SmartDashboard.putNumber("Camera To Use", cameraToUse.tv());
        turninPID.setSetpoint(turnToClosest());
        if (cameraToUse.tv() == 1 && turnedTo(turnToClosest())) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    -(0.38 - cameraToUse.ta()) * 4,
                    dynamicSpeedY(),
                    MathUtil.applyDeadband(
                            -turninPID.calculate(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()),
                            0.1), // * IO.maxTurnSpeed,
                    swerveSubsystem.getRotation2d());

        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    MathUtil.applyDeadband(
                            -turninPID.calculate(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()),
                            0.1), // * IO.maxTurnSpeed,
                    swerveSubsystem.getRotation2d());
        }

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kinematics.m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // Output each module states to wheels
        swerveSubsystem.setModuleState(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        f_LimeLight.setLED(1);
        b_LimeLight.setLED(1);
        b_LimeLight.setPipeline(1);
        f_LimeLight.setPipeline(1);

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeToAlign);
    }

    public void updateUsedCamera() {
        if (swerveSubsystem.getSide() == 1) {
            cameraToUse = b_LimeLight;
        } else {
            cameraToUse = f_LimeLight;
        }
    }

    public double turnToClosest() {
        if (Math.abs(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()) > 90) {
            return 180;
        } else
            return 0;

    }

    public double dynamicSpeedY() {
        double error = ((-2.5) - cameraToUse.ty());
        double deadband;
        boolean atPOse = false;
        if (Math.abs(error) > 0.1) {
            deadband = 0.2;
        } else {
            deadband = 0.05;
        }

        return MathUtil.applyDeadband(error * 0.1, deadband);

    }

    public boolean turnedTo(double angle) {
        double turningError = -Math.abs(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees())
                + (Math.abs(angle));

        if (turningError < 5) {
            return true;
        } else {
            return false;
        }

    }

}