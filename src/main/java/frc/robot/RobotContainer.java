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
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SubsystemCommands.DefualtDriveCommand;

public class RobotContainer {

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
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

PathPlannerTrajectory pathGroup = PathPlanner.loadPath("Go to Cone", new PathConstraints(0.5, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    swerveSubsystem::getPose, // Pose2d supplier
    swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    DrivebaseConstants.m_kinematics, // SwerveDriveKinematics
    new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.followPath(pathGroup);
return aBalance;
}
}