package frc.robot.Auto.Commands.drivebaseCommands.AutoCommands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.subsystems.swerveSubsystem;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Custom PathPlanner version of SwerveControllerCommand */
public class followTrajectoryCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final Timer timer2 = new Timer();
  private static final swerveSubsystem swerve = RobotContainer.swerveSubsystem;
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier = swerve::getPose;
  private final SwerveDriveKinematics kinematics = drivebaseConstants.kinematics.m_kinematics;
  private final CustomFollower controller = new CustomFollower(new PIDController(1.8, 0.00, 0.4),
      new PIDController(1.8, 0.00, 0.4), new PIDController(5, 0, 0));;
  private final Consumer<SwerveModuleState[]> outputModuleStates = swerve::setModuleState;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds = swerve::setChassisSpeeds;
  private final boolean useKinematics = false;
  private final boolean useAllianceColor = false;
  private final boolean isFirstPath;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError = followTrajectoryCommand::defaultLogError;

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the
   * provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the output to zero upon completion of
   * the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param poseSupplier       A function that supplies the robot pose - use one
   *                           of the odometry classes
   *                           to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller for the
   *                           robot's x position.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param useAllianceColor   Should the path states be automatically transformed
   *                           based on alliance
   *                           color? In order for this to work properly, you MUST
   *                           create your path on the blue side of
   *                           the field.
   * @param requirements       The subsystems to require.
   */
  public followTrajectoryCommand(
      PathPlannerTrajectory trajectory, boolean isFirstPath) {
    this.trajectory = trajectory;
    this.isFirstPath = isFirstPath;
    addRequirements(RobotContainer.swerveSubsystem);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  @Override
  public void initialize() {
    if (isFirstPath) {
      swerve.resetOdometry(trajectory.getInitialHolonomicPose());
    }
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
          trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    timer.reset();
    timer.start();

    controller.setTolerance(new Pose2d(new Translation2d( 0.04, 0.04), Rotation2d.fromDegrees(1) ));

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = this.poseSupplier.get();

    PathPlannerServer.sendPathFollowingData(
        new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        currentPose);

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

    if (this.useKinematics) {
      SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

      this.outputModuleStates.accept(targetModuleStates);
    } else {
      this.outputChassisSpeeds.accept(targetChassisSpeeds);
    }

    if (logTargetPose != null) {
      logTargetPose.accept(
          new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.holonomicRotation));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }

    defaultLogError(currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
        currentPose.getRotation().minus(desiredState.holonomicRotation));
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
      if (useKinematics) {
        this.outputModuleStates.accept(
            this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
      } else {
        this.outputChassisSpeeds.accept(new ChassisSpeeds());
      }
    }
  }

  @Override
  public boolean isFinished() {
    return atReferenceTimed(0.2) || this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds()+3);

  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber("PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default
   * configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory
   *                            representing the
   *                            active path. This will be called whenever a
   *                            PPSwerveControllerCommand starts
   * @param logTargetPose       Consumer that accepts a Pose2d representing the
   *                            target pose while path
   *                            following
   * @param logSetpoint         Consumer that accepts a ChassisSpeeds object
   *                            representing the setpoint
   *                            speeds
   * @param logError            BiConsumer that accepts a Translation2d and
   *                            Rotation2d representing the error
   *                            while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    followTrajectoryCommand.logActiveTrajectory = logActiveTrajectory;
    followTrajectoryCommand.logTargetPose = logTargetPose;
    followTrajectoryCommand.logSetpoint = logSetpoint;
    followTrajectoryCommand.logError = logError;
  }

  public boolean atReferenceTimed(double time){
    if(controller.atReference()){
      timer2.start();
    }else{
      timer2.reset();
      timer2.stop();
    }
    return timer2.hasElapsed(time);
  }

}
