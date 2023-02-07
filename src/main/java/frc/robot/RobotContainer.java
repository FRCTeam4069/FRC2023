package frc.robot;

import java.util.List;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.controls.DefualtDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final Gyro gyro = new Gyro();
    public static final LimeLight limelight = new LimeLight();

    public void PlayMusic(String Filename){
        Orchestra orchestra = new Orchestra();
        orchestra.loadMusic(Filename);
        orchestra.addInstrument(swerveSubsystem.getFRDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getFLDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getBRDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getBLDriveMotor());
        orchestra.play();
    }

    private final XboxController Controller1 = new XboxController(0);

       public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new DefualtDriveCommand(
                swerveSubsystem,
                () -> Controller1.getLeftX(),
                () -> -Controller1.getLeftY(),
                () -> -Controller1.getRightX(),
                () -> !Controller1.getAButton(),
                () -> Controller1.getRightBumper()));

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        new Button(Controller1::getAButton).whenPressed(() -> swerveSubsystem.resetGyro());
        new Button(Controller1::getBButton).whenActive(() -> swerveSubsystem.zeroAllWhels());
    }

    public Command getAutonomousCommand() {
// 1. Create trajectory settings
TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    1,
    2)
            .setKinematics(Constants.DrivebaseConstants.m_kinematics);
// 2. Generate trajectory
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
            new Translation2d(1, 0),
            new Translation2d(0, 0)),
    new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(.5, 0, 0);
PIDController yController = new PIDController(.5, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
    3, 0, 0, DrivebaseConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

// 4. Construct command to follow trajectory
SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory,
    swerveSubsystem::getPose,
    DrivebaseConstants.m_kinematics,
    xController,
    yController,
    thetaController,
    swerveSubsystem::setModuleStates,
    swerveSubsystem);

// 5. Add some init and wrap-up, and return everything
return new SequentialCommandGroup(
    new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
    swerveControllerCommand,
    new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}