package frc.robot;

import java.util.HashMap;
import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.swerveSubsystem;
import frc.robot.Auto.Commands.Presets.AUTOgourdCone;
import frc.robot.subsystems.ledHelper;
import frc.robot.Auto.Commands.Presets.HighPoseS1;
import frc.robot.Auto.Commands.Presets.HighPoseS2;
import frc.robot.Auto.Commands.Presets.HomePose;
import frc.robot.Auto.Commands.Presets.HumanPlayerCONE;
import frc.robot.Auto.Commands.Presets.HumanPlayerCUBE;
import frc.robot.Auto.Commands.Presets.MidPoseS1;
import frc.robot.Auto.Commands.Presets.MidPoseS2;
import frc.robot.Auto.Commands.Presets.ShootCubeL3;
import frc.robot.Auto.Commands.Presets.ledCommand;
import frc.robot.Auto.Commands.Presets.scoreThenHome;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.drivebaseCommands.defaultDriveCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.autoWristParallel;
import frc.robot.Auto.Commands.intakeAndWristCommands.defaultIntakeCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.defaultWristCommand;
import frc.robot.Auto.Commands.intakeAndWristCommands.intakeToPose;
import frc.robot.Auto.Commands.intakeAndWristCommands.wristToPosition;
import frc.robot.Auto.routines.BLUE_LONG;
import frc.robot.Auto.routines.BLUE_SHORT;
import frc.robot.Auto.routines.FirstRoutine;
import frc.robot.Auto.routines.MiddlePathL3CUBUE;
import frc.robot.Auto.routines.Middle_Path_0cones;
import frc.robot.Auto.routines.RED_LONG;
import frc.robot.Auto.routines.RED_SHORT;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.cameraHelper;
import frc.robot.subsystems.AutoSelecter;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.wristSubsystem;
import frc.robot.subsystems.ledHelper.Colours;

public class RobotContainer {

    public static final swerveSubsystem swerveSubsystem = new swerveSubsystem(-90);
    public static final armSubsystem arm = new armSubsystem();
    public static final intakeSubsystem intake = new intakeSubsystem();
    public static final ledHelper leds = new ledHelper(9);
    //public static final LimeLight1 limelight = new LimeLight1();
    public static final cameraHelper frontLimeLight = new cameraHelper("Front Camera","http://10.40.69.39:5800","limelight-fcam");
    public static final cameraHelper backLimeLight = new cameraHelper("Back Camera","http://10.40.69.70:5801","limelight-bcam");
    public static final wristSubsystem wrist = new wristSubsystem();

    public static final Middle_Path_0cones testRoutine = new Middle_Path_0cones();
    public static final AutoSelecter autoSelecter = new AutoSelecter();
    public static HashMap<String, Command> eventMap = new HashMap<>();

    public double pressed = 0;
    public IO.state state = IO.state.START;

    public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            drivebaseConstants.kinematics.m_kinematics, // SwerveDriveKinematics
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and
                                             // Y PID controllers)
            new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                             // controller)
            swerveSubsystem::setModuleState, // Module states consumer used to output to the drive subsystem
            eventMap,
            false, // Should the path be automatically mirrored depending on alliance color.
                   // Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following
                            // commands
    );

    public void PlayMusic(String Filename) {
        Orchestra orchestra = new Orchestra();
        orchestra.loadMusic(Filename);
        orchestra.addInstrument(swerveSubsystem.getFRDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getFLDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getBRDriveMotor());
        orchestra.addInstrument(swerveSubsystem.getBLDriveMotor());
        orchestra.play();
    }

    public final static XboxController Controller1 = new XboxController(0);
    public final static XboxController Controller2 = new XboxController(1);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new defaultDriveCommand(
                swerveSubsystem,
                () -> Controller1.getLeftX(),
                () -> Controller1.getLeftY(),
                () -> Controller1.getRightX(),
                () -> Controller1.getYButton(),
                () -> Controller1.getLeftBumper(),
                () -> Controller1.getRightBumper()));

        arm.setDefaultCommand(new DefaultArmCommand(
                () -> Controller2.getRightY(),
                () -> Controller2.getLeftY(),
                () -> swerveSubsystem.getSide()));

        intake.setDefaultCommand(new defaultIntakeCommand(
                () -> Controller2.getLeftBumper(),
                () -> Controller2.getRightBumper(),
                () -> Controller2.getPOV()));

        wrist.setDefaultCommand(new defaultWristCommand(
            () -> Controller2.getRightTriggerAxis(), 
            () -> Controller2.getLeftTriggerAxis()));

        // intake.setDefaultCommand(new autoWristPose());

        leds.setDefaultCommand(new ledCommand(
            () -> intake.coneInRange,
            () -> Controller1.getYButton()
        ));

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        new Trigger(Controller1::getAButton).whileTrue(swerveSubsystem.resetGyroCommmand());
        new Trigger(Controller2::getLeftStickButton).whileTrue(arm.flaseLimit());
        new Trigger(Controller2::getRightStickButton).whileTrue(arm.trueLimit());
        if (IO.LastState == IO.state.HIGH || IO.LastState == IO.state.MID) {
            new Trigger(Controller2::getAButton).onTrue(new scoreThenHome());
        } else {
            new Trigger(Controller2::getAButton).onTrue(new HomePose());
        }
        new Trigger(Controller2::getYButton).onTrue(new HighPoseS1()).onFalse(new HighPoseS2());
        new Trigger(Controller2::getBButton).onTrue(new HumanPlayerCONE());
        new Trigger(Controller2::getBackButton).onTrue(new HumanPlayerCUBE());
        new Trigger(Controller2::getXButton).onTrue(new MidPoseS1()).onFalse(new MidPoseS2());
        new POVButton(Controller2, 270).whileTrue(new autoWristParallel());
        new POVButton(Controller2, 90).onTrue(new wristToPosition(-70, 10, 0.5, 1));
        new POVButton(Controller2, 180).whileTrue(new intakeToPose(12.5, 0.1, 1, .6));
        //new Trigger( ()-> intake.coneInRange ).onFalse(new rumbleBothControllers(0.8, 1));
        
        new Trigger(Controller1::getXButton).onTrue(leds.setDefaultColor(Colours.BLUE));
        new Trigger(Controller1::getBButton).onTrue(leds.setDefaultColor(Colours.ERROR_YELLOw));

    }



    public Command getAutonomousCommand() {
        //return autoBuilder.fullAuto(pathGroup);
        int autoIndex = autoSelecter.getSelected();
        switch (autoIndex) {
        case 0:
        return new MiddlePathL3CUBUE();
        case 1:
        return new RED_LONG();
        case 2:
        return new RED_SHORT();
        case 3:
        return new BLUE_LONG();
        case 4:
        return new BLUE_SHORT();
        case 5:
        return new ShootCubeL3();
        case 6:
        return new InstantCommand();
        case 7:
        return new FirstRoutine();
        //return new followTrajectoryCommand(PathPlanner.loadPath("TEST", new PathConstraints(2, 2)), true);
        default:
        SmartDashboard.putString("Auto Selected:", "INVALID");
        return new InstantCommand();

        }
    }
}