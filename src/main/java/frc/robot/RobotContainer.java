package frc.robot;

import java.util.HashMap;
import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Auto.Commands.Drivebase.EXPERIMENTALautoBalance;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SubsystemCommands.DefualtDriveCommand;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final ExampleSubsystem exmaple = new ExampleSubsystem();
    public static final Gyro gyro = new Gyro();
    public static final LimeLight limelight = new LimeLight();
    public EXPERIMENTALautoBalance aBalance = new EXPERIMENTALautoBalance(swerveSubsystem, gyro);

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
    return aBalance;
    }
}