package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants.ModuleConstants;
import frc.robot.Constants.drivebaseConstants.deviceIDs;
import frc.robot.Constants.drivebaseConstants.kinematics;

public class SwerveSubsystem extends SubsystemBase {

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    public final Module FRSwerveModule = new Module(
            deviceIDs.FR_DRIVE_MOTOR,
            deviceIDs.FR_STEER_MOTOR,
            deviceIDs.FR_DRIVE_MOTOR_REVERSED,
            deviceIDs.FR_STEER_MOTOR_REVERSED,
            deviceIDs.FR_STEER_ENCODER,
            deviceIDs.FR_STEER_OFFSET,
            deviceIDs.FR_STEER_ENCODER_REVERSED);

    private final Module FLSwerveModule = new Module(
            deviceIDs.FL_DRIVE_MOTOR,
            deviceIDs.FL_STEER_MOTOR,
            deviceIDs.FL_DRIVE_MOTOR_REVERSED,
            deviceIDs.FL_STEER_MOTOR_REVERSED,
            deviceIDs.FL_STEER_ENCODER,
            deviceIDs.FL_STEER_OFFSET,
            deviceIDs.FL_STEER_ENCODER_REVERSED);

    private final Module BRSwerveModule = new Module(
            deviceIDs.BR_DRIVE_MOTOR,
            deviceIDs.BR_STEER_MOTOR,
            deviceIDs.BR_DRIVE_MOTOR_REVERSED,
            deviceIDs.BR_STEER_MOTOR_REVERSED,
            deviceIDs.BR_STEER_ENCODER,
            deviceIDs.BR_STEER_OFFSET,
            deviceIDs.BR_STEER_ENCODER_REVERSED);

    private final Module BLSwerveModule = new Module(
            deviceIDs.BL_DRIVE_MOTOR,
            deviceIDs.BL_STEER_MOTOR,
            deviceIDs.BL_DRIVE_MOTOR_REVERSED,
            deviceIDs.BL_STEER_MOTOR_REVERSED,
            deviceIDs.BL_STEER_ENCODER,
            deviceIDs.BL_STEER_OFFSET,
            deviceIDs.BL_STEER_ENCODER_REVERSED

    );

    private GenericEntry BLp = tab.add("BLSwerveModule", BLSwerveModule.getPosition().toString()).getEntry();
    private GenericEntry BRp = tab.add("BRSwerveModule", BLSwerveModule.getPosition().toString()).getEntry();
    private GenericEntry FLp = tab.add("FLSwerveModule", BLSwerveModule.getPosition().toString()).getEntry();
    private GenericEntry FRp = tab.add("FRSwerveModule", BLSwerveModule.getPosition().toString()).getEntry();
    private GenericEntry BLs = tab.add("Back Left State", BLSwerveModule.getState()).getEntry();
    private GenericEntry BRs = tab.add("Back Left State", BLSwerveModule.getState()).getEntry();
    private GenericEntry Fls = tab.add("Back Left State", BLSwerveModule.getState()).getEntry();
    private GenericEntry FRs = tab.add("Back Left State", BLSwerveModule.getState()).getEntry();

    public static final Gyro gyro = RobotContainer.gyro;
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics.m_kinematics, getRotation2d(),
            getModulePositions());

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                FRSwerveModule.getPosition(),
                FLSwerveModule.getPosition(),
                BRSwerveModule.getPosition(),
                BLSwerveModule.getPosition(),
        };
    }

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
            } catch (Exception e) {
            }
        }).start();

    }

    public void resetGyro() {
        gyro.setYaw(-90);
    }

    public Gyro getGyro() {
        return gyro;
    }

    public void zeroAllWhels() {
        FRSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        BRSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        FLSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        BLSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));

    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
                FRSwerveModule.getPosition(),
                FLSwerveModule.getPosition(),
                BRSwerveModule.getPosition(),
                BLSwerveModule.getPosition(),
        }, new Pose2d(0, 0, new Rotation2d(gyro.getHeading())));
    }

    public void stopModules() {
        FRSwerveModule.stop();
        FLSwerveModule.stop();
        BRSwerveModule.stop();
        BLSwerveModule.stop();
    }

    public CommandBase resetGyroCommmand() {
        return this.runOnce(() -> resetGyro());
    }

    public void setModuleStates(SwerveModuleState[] ModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        FLSwerveModule.setDesiredState(ModuleStates[0]);
        FRSwerveModule.setDesiredState(ModuleStates[1]);
        ;
        BLSwerveModule.setDesiredState(ModuleStates[2]);
        BRSwerveModule.setDesiredState(ModuleStates[3]);

    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), new SwerveModulePosition[] {
                FRSwerveModule.getPosition(),
                FLSwerveModule.getPosition(),
                BRSwerveModule.getPosition(),
                BLSwerveModule.getPosition() });

        if (IO.PrintSwerveData) {
            tab.add("BLSwerveModule", BLSwerveModule.getPosition().toString());
            tab.add("BRSwerveModule", BLSwerveModule.getPosition().toString());
            tab.add("FLSwerveModule", BLSwerveModule.getPosition().toString());
            tab.add("FRSwerveModule", BLSwerveModule.getPosition().toString());
            tab.add("Back Left State", BLSwerveModule.getState());
            tab.add("Back Left State", BLSwerveModule.getState());
            tab.add("Back Left State", BLSwerveModule.getState());
            tab.add("Back Left State", BLSwerveModule.getState());
        }

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

}
