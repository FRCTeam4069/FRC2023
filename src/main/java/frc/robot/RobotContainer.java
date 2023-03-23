package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.Commands.armCommands.DefaultArmCommand;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HighPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HumanPlayerPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.MidPose;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.scoreThenHome;
import frc.robot.Auto.Commands.drivebaseCommands.defaultDriveCommand;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;
import frc.robot.Auto.Commands.drivebaseCommands.leaveCommunity;
import frc.robot.Auto.Commands.intakeCommands.DefaultIntakeCommand;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.Auto.Commands.intakeCommands.OpenIntake;
import frc.robot.Auto.Commands.intakeCommands.wristToPosition;
import frc.robot.Auto.routines.Middle_Path_0cones;
import frc.robot.Auto.routines.PlaceCubeAndArmDown;
import frc.robot.Auto.routines.TestPathPlannerPath;
import frc.robot.Auto.routines.placeCubeL3;
import frc.robot.Constants.AutoValues;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.AutonSelect;
import frc.robot.subsystems.Debugger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(-90);
    public static final armSubsystem arm = new armSubsystem();
    public static final Intake intake = new Intake();
    public static final LimeLight limelight = new LimeLight();

    public static final Middle_Path_0cones testRoutine = new Middle_Path_0cones();
    public static final AutonSelect autoSelecter = new AutonSelect();
    public static final Debugger db = new Debugger();
    public followTrajectoryCommand fTrajectoryCommand;
    private static final PathPlannerTrajectory pathGroup = PathPlanner.loadPath("Two Items", new PathConstraints(4,3));
    public static HashMap<String, Command> eventMap = new HashMap<>();


    public double pressed = 0;

    public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerveSubsystem::getPose, // Pose2d supplier
        swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        drivebaseConstants.kinematics.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
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

    private final XboxController Controller1 = new XboxController(0);
    private final XboxController Controller2 = new XboxController(1);

    public RobotContainer() {

    eventMap.put("CloseIntake", new timeBasedIntake(0.5, .25));
    eventMap.put("CUBE_L3", new placeCubeL3());
    eventMap.put("ARM_DOWN", new HomePose());

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

        intake.setDefaultCommand(new DefaultIntakeCommand(
                () -> Controller2.getRightTriggerAxis(),
                () -> Controller2.getLeftTriggerAxis(),
                () -> Controller2.getLeftBumper(),
                () -> Controller2.getRightBumper()));

        // intake.setDefaultCommand(new autoWristPose());

        configureButtonBindings();

    }

    private void configureButtonBindings() {
         new Trigger(Controller1::getAButton).whileTrue(swerveSubsystem.resetGyroCommmand());
        // new Trigger(Controller2::getStartButton).whileTrue(arm.runOnce(() ->
        // arm.setZero()));
        new Trigger(Controller2::getLeftStickButton).whileTrue(arm.flaseLimit());
        new Trigger(Controller2::getRightStickButton).whileTrue(arm.trueLimit());
        if (IO.LastState == IO.state.HIGH || IO.LastState == IO.state.MID) {
            new Trigger(Controller2::getAButton).onTrue(new scoreThenHome());
        } else {
            new Trigger(Controller2::getAButton).onTrue(new HomePose());
        }
        new Trigger(Controller2::getYButton)
        .onTrue(new HighPose())
        .onFalse(new SequentialCommandGroup(new OpenIntake().andThen(new HomePose())));
        new Trigger(Controller2::getBButton).onTrue(new HumanPlayerPose());
        new Trigger(Controller2::getXButton).onTrue(new MidPose());
         

        
    }



    public Command getAutonomousCommand() {
        return autoBuilder.fullAuto(pathGroup);
        // int autoIndex = autoSelecter.getSelected();
        // switch (autoIndex) {
        //     case 0:
        //         return new Middle_Path_0cones();
        //     case 1:
        //         return new leaveCommunity(-60, () -> swerveSubsystem.odometry.getPoseMeters().getX());
        //     case 2:
        //         return new leaveCommunity(-75, () -> swerveSubsystem.odometry.getPoseMeters().getX());
        //     case 3:
        //         return new placeCubeL3();
        //     case 4:
        //         return new auto;   
        //     case 5:
        //         return new TestPathPlannerPath();
        //     case 6:
        //         return new InstantCommand();
        //     default:
        //         SmartDashboard.putString("Auto Selected:", "INVALID");
        //         return new InstantCommand();

        // }
    }
}