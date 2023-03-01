package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.drivebaseCommands.DefualtDriveCommand;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.intakeCommands.DefaultIntakeCommand;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.subsystemCommands.resetGyro;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public armSubsystem arm = new armSubsystem();
    public Intake intake = new Intake();
    public Gyro gyro = new Gyro();

    public resetGyro resetGyro = new resetGyro(swerveSubsystem);
    public LimeLight limelight = new LimeLight();
    public autoBalance aBalance = new autoBalance(swerveSubsystem, gyro);
    public followTrajectoryCommand fTrajectoryCommand;
    

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
    private final XboxController Controller2 = new XboxController(1);

       public RobotContainer() {
        fTrajectoryCommand = new followTrajectoryCommand(swerveSubsystem, PathPlanner.loadPath("Go to Cone", new PathConstraints(0.5, 1)) , true);
        swerveSubsystem.setDefaultCommand(new DefualtDriveCommand(
                swerveSubsystem,
                () -> Controller1.getLeftX(),
                () -> -Controller1.getLeftY(),
                () -> -Controller1.getRightX(),
                () -> !Controller1.getAButton(),
                () -> Controller1.getXButton()));
                
        arm.setDefaultCommand(new DefaultArmCommand(
            arm, 
            () -> Controller2.getRightY(),
            () -> Controller2.getLeftY(),
            () -> Controller2.getLeftBumper(),
            () -> Controller2.getRightBumper(),
            () -> Controller2.getAButton(),
            () -> Controller2.getStartButton(),
            () -> gyro.getHeading()));

        intake.setDefaultCommand( new DefaultIntakeCommand(
            intake,
            () -> Controller2.getRightTriggerAxis(),
            () -> Controller2.getLeftTriggerAxis(),
            () -> Controller2.getXButton(),
            () -> Controller2.getBButton()));        
        
        configureButtonBindings();

    }

    private void configureButtonBindings() {
         new Trigger(Controller1::getAButton).whileTrue(resetGyro);
        
    }

    public Command getAutonomousCommand() {
    return aBalance;
    }
}