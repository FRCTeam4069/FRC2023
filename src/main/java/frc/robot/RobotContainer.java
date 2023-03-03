package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.armCommands.moveToPose;
import frc.robot.Auto.Commands.drivebaseCommands.DefualtDriveCommand;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.drivebaseCommands.leaveCommunity;
import frc.robot.Auto.Commands.intakeCommands.DefaultIntakeCommand;
import frc.robot.Auto.routines.Middle_Path_0cones;
import frc.robot.Constants.drivebaseConstants.deviceIDs;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.AutonSelect;
import frc.robot.subsystems.Debugger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Gyro;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final armSubsystem arm = new armSubsystem();
    public static final Intake intake = new Intake();
    public static final LimeLight limelight = new LimeLight();
    public static final Gyro gyro = new Gyro(deviceIDs.PIGEON_ID);

    
    public static final autoBalance aBalance = new autoBalance();
    public static final Middle_Path_0cones testRoutine = new Middle_Path_0cones();
    public static final AutonSelect autoSelecter = new AutonSelect();
    public static final Debugger db = new Debugger();
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
        swerveSubsystem.setDefaultCommand(new DefualtDriveCommand(
                swerveSubsystem,
                () -> Controller1.getLeftX(),
                () -> -Controller1.getLeftY(),
                () -> -Controller1.getRightX(),
                () -> !Controller1.getAButton(),
                () -> Controller1.getXButton()));
                
        arm.setDefaultCommand(new DefaultArmCommand(
            () -> Controller2.getRightY(),
            () -> Controller2.getLeftY()));

        intake.setDefaultCommand( new DefaultIntakeCommand(
            intake,
            () -> Controller2.getRightTriggerAxis(),
            () -> Controller2.getLeftTriggerAxis(),
            () -> Controller2.getXButton(),
            () -> Controller2.getBButton()));        
        
        configureButtonBindings();

    }

    private void configureButtonBindings() {
         new Trigger(Controller1::getAButton).whileTrue(swerveSubsystem.resetGyroCommmand());
         new Trigger(Controller2::getStartButton).whileTrue(arm.runOnce(()-> arm.setZero()));
         new Trigger(Controller2::getLeftStickButton).whileTrue(arm.flaseLimit());
         new Trigger(Controller2::getRightStickButton).whileTrue(arm.trueLimit());
         new Trigger(Controller2::getLeftBumper).whileTrue(new moveToPose(55,-1));
         new Trigger(Controller2::getRightBumper).whileTrue(new moveToPose(-55,-1));
         
        
    }

    public Command getAutonomousCommand() {
        int autoIndex = autoSelecter.getSelected();
        switch(autoIndex){
            case 0 :
            return new Middle_Path_0cones();
            case 5: 
            return new leaveCommunity();
            default:
            SmartDashboard.putString("Auto Selected:", "INVALID");
            return new InstantCommand(); 
            
        }
    }
}