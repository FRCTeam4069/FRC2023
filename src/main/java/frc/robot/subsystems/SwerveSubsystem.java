package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.Constants.drivebaseConstants.ModuleConstants;
import frc.robot.Constants.drivebaseConstants.deviceIDs;
import frc.robot.Constants.drivebaseConstants.kinematics;

public class swerveSubsystem extends SubsystemBase {
    public final double heading;
    public swerveSubsystem(double heading) {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
            } catch (Exception e) {
            }
        }).start();

        resetOdometry(new Pose2d());

        this.heading = heading;

    }

    public final swerveModule FRSwerveModule = new swerveModule(
            deviceIDs.FR_DRIVE_MOTOR,
            deviceIDs.FR_STEER_MOTOR,
            deviceIDs.FR_DRIVE_MOTOR_REVERSED,
            deviceIDs.FR_STEER_MOTOR_REVERSED,
            deviceIDs.FR_STEER_ENCODER,
            deviceIDs.FR_STEER_OFFSET,
            deviceIDs.FR_STEER_ENCODER_REVERSED);

    public final swerveModule FLSwerveModule = new swerveModule(
            deviceIDs.FL_DRIVE_MOTOR,
            deviceIDs.FL_STEER_MOTOR,
            deviceIDs.FL_DRIVE_MOTOR_REVERSED,
            deviceIDs.FL_STEER_MOTOR_REVERSED,
            deviceIDs.FL_STEER_ENCODER,
            deviceIDs.FL_STEER_OFFSET,
            deviceIDs.FL_STEER_ENCODER_REVERSED);

    public final swerveModule BRSwerveModule = new swerveModule(
            deviceIDs.BR_DRIVE_MOTOR,
            deviceIDs.BR_STEER_MOTOR,
            deviceIDs.BR_DRIVE_MOTOR_REVERSED,
            deviceIDs.BR_STEER_MOTOR_REVERSED,
            deviceIDs.BR_STEER_ENCODER,
            deviceIDs.BR_STEER_OFFSET,
            deviceIDs.BR_STEER_ENCODER_REVERSED);

    public final swerveModule BLSwerveModule = new swerveModule(
            deviceIDs.BL_DRIVE_MOTOR,
            deviceIDs.BL_STEER_MOTOR,
            deviceIDs.BL_DRIVE_MOTOR_REVERSED,
            deviceIDs.BL_STEER_MOTOR_REVERSED,
            deviceIDs.BL_STEER_ENCODER,
            deviceIDs.BL_STEER_OFFSET,
            deviceIDs.BL_STEER_ENCODER_REVERSED

    );

    public static final gyroHelper gyro = new gyroHelper(deviceIDs.PIGEON_ID);
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics.m_kinematics, getRotation2d(),
            getModulePositions());
            

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                FLSwerveModule.getPosition(),
                FRSwerveModule.getPosition(),
                BLSwerveModule.getPosition(),
                BRSwerveModule.getPosition(),
        };
    }

    public SwerveModuleState[] angleModules(double degree){
        return new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(degree)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(degree)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(degree)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(degree))};

    }


    public void resetGyro() {
        //gyro.setYaw(0);
        gyro.setYaw(0);
    }

    public gyroHelper getGyro() {
        return gyro;
    }

    public void zeroAllWhels() {
        FLSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        FRSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        BLSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        BRSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));

    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getHeading());
    }


    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d newPose2d) {
        resetGyro();
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
                FLSwerveModule.getPosition(),
                FRSwerveModule.getPosition(),
                BLSwerveModule.getPosition(),
                BRSwerveModule.getPosition(),
        }, newPose2d);

        
    }

    public void stopModules() {
        FLSwerveModule.stop();
        FRSwerveModule.stop();
        BLSwerveModule.stop();
        BRSwerveModule.stop();
    }

    public CommandBase resetGyroCommmand() {
        return this.runOnce(() -> resetGyro());
    }


    public void setModuleState(SwerveModuleState[] ModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        FLSwerveModule.setDesiredState(ModuleStates[0]);
        FRSwerveModule.setDesiredState(ModuleStates[1]);
        BLSwerveModule.setDesiredState(ModuleStates[2]);
        BRSwerveModule.setDesiredState(ModuleStates[3]);

    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        setModuleState(kinematics.m_kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public double getSide() {
        if (Math.abs(odometry.getPoseMeters().getRotation().getDegrees()) > 90) {
            return -1;
        }else return 1;
    }
   


    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putString("Odometry", odometry.getPoseMeters().toString());
        SmartDashboard.putString("Last Known State", IO.LastState.toString());
        SmartDashboard.putNumber("Robot Side", getSide());

        odometry.update(getRotation2d(), new SwerveModulePosition[] {
                FLSwerveModule.getPosition(),
                FRSwerveModule.getPosition(),
                BLSwerveModule.getPosition(),
                BRSwerveModule.getPosition() });
            }

    public TalonFX getFRDriveMotor() {
        return FRSwerveModule.getDriveMotor();
    }

    public TalonFX getFLDriveMotor() {
        return FLSwerveModule.getDriveMotor();
    }

    public TalonFX getBRDriveMotor() {
        return BRSwerveModule.getDriveMotor();
    }

    public TalonFX getBLDriveMotor() {
        return BLSwerveModule.getDriveMotor();
    }


    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        drivebaseConstants.kinematics.m_kinematics, // SwerveDriveKinematics
                        new PIDController(1.9, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                      // will only use feedforwards.
                        new PIDController(1.9, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(-0.3, 0.00, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                    // only use feedforwards.
                        this::setModuleState, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color.
                               // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }

}
