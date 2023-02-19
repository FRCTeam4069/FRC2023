package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.drivebaseCommands.DefualtDriveCommand;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public armSubsystem arm = new armSubsystem();
    public Gyro gyro = new Gyro();
    public LimeLight limelight = new LimeLight();
    public autoBalance aBalance = new autoBalance(swerveSubsystem, gyro);

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
                
        arm.setDefaultCommand(new DefaultArmCommand(
            arm, 
            () -> Controller1.getRightTriggerAxis(),
            () -> Controller1.getLeftTriggerAxis()));
        
        
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