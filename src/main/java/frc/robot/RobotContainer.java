package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HighPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HumanPlayerPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.MidPose;
import frc.robot.Auto.Commands.drivebaseCommands.DefualtDriveCommand;
import frc.robot.Auto.Commands.drivebaseCommands.autoBalance;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.drivebaseCommands.leaveCommunity;
import frc.robot.Auto.Commands.intakeCommands.DefaultIntakeCommand;
import frc.robot.Auto.Commands.intakeCommands.autoWristPose;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Auto.routines.Middle_Path_0cones;
import frc.robot.Auto.routines.PlaceCubeAndArmDown;
import frc.robot.Auto.routines.placeCube;
import frc.robot.Constants.IO;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.AutonSelect;
import frc.robot.subsystems.Debugger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final armSubsystem arm = new armSubsystem();
    public static final Intake intake = new Intake();
    public static final LimeLight limelight = new LimeLight();

    
    public static final autoBalance aBalance = new autoBalance();
    public static final Middle_Path_0cones testRoutine = new Middle_Path_0cones();
    public static final AutonSelect autoSelecter = new AutonSelect();
    public static final Debugger db = new Debugger();
    public followTrajectoryCommand fTrajectoryCommand;


    public void PlayMusic(String Filename) {
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
                () -> Controller1.getAButton(),
                () -> Controller1.getRightBumper()));

        arm.setDefaultCommand(new DefaultArmCommand(
                () -> Controller2.getRightY(),
                () -> Controller2.getLeftY(),
                ()-> swerveSubsystem.getSide()));

        intake.setDefaultCommand(new DefaultIntakeCommand(
                () -> Controller2.getRightTriggerAxis(),
                () -> Controller2.getLeftTriggerAxis(),
                () -> Controller2.getLeftBumper(),
                () -> Controller2.getRightBumper()));

        //intake.setDefaultCommand(new autoWristPose());

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        new Trigger(Controller1::getAButton).whileTrue(swerveSubsystem.resetGyroCommmand());
        //new Trigger(Controller2::getStartButton).whileTrue(arm.runOnce(() -> arm.setZero()));
        new Trigger(Controller2::getLeftStickButton).whileTrue(arm.flaseLimit());
        new Trigger(Controller2::getRightStickButton).whileTrue(arm.trueLimit());
        new Trigger(Controller2::getAButton).onTrue(new HomePose());
        new Trigger(Controller2::getYButton).onTrue(new HighPose());
        new Trigger(Controller2::getXButton).onTrue(new HumanPlayerPose());
        new Trigger(Controller2::getBButton).onTrue(new MidPose());
        new Trigger(Controller2::getStartButton).whileTrue(new autoWristPose());
    }

    public Command getAutonomousCommand() {
        int autoIndex = autoSelecter.getSelected();
        switch(autoIndex){
            case 0 :
            return new Middle_Path_0cones();
            case 1 :
            return new leaveCommunity(-60, ()-> swerveSubsystem.odometry.getPoseMeters().getX());
            case 2: 
            return new leaveCommunity(-75, ()-> swerveSubsystem.odometry.getPoseMeters().getX());
            case 3 :
            return new placeCube();
            case 4 :
            return new PlaceCubeAndArmDown();

            case 5 :
            return new InstantCommand();
            case 6 :
            return new InstantCommand();
            default:
                SmartDashboard.putString("Auto Selected:", "INVALID");
                return new InstantCommand();

        }
    }
}