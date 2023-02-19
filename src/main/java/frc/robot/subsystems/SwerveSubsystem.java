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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Auto.Commands.Drivebase.EXPERIMENTALautoBalance;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase{

    public final Module FRSwerveModule = new Module(
        DrivebaseConstants.FR_DRIVE_MOTOR,
        DrivebaseConstants.FR_STEER_MOTOR,
        DrivebaseConstants.FR_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.FR_STEER_MOTOR_REVERSED,
        DrivebaseConstants.FR_STEER_ENCODER,
        DrivebaseConstants.FR_STEER_OFFSET,
        DrivebaseConstants.FR_STEER_ENCODER_REVERSED
    );

    private final Module FLSwerveModule = new Module(
        DrivebaseConstants.FL_DRIVE_MOTOR,
        DrivebaseConstants.FL_STEER_MOTOR,
        DrivebaseConstants.FL_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.FL_STEER_MOTOR_REVERSED,
        DrivebaseConstants.FL_STEER_ENCODER,
        DrivebaseConstants.FL_STEER_OFFSET,
        DrivebaseConstants.FL_STEER_ENCODER_REVERSED
    );

    private final Module BRSwerveModule = new Module(
        DrivebaseConstants.BR_DRIVE_MOTOR,
        DrivebaseConstants.BR_STEER_MOTOR,
        DrivebaseConstants.BR_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.BR_STEER_MOTOR_REVERSED,
        DrivebaseConstants.BR_STEER_ENCODER,
        DrivebaseConstants.BR_STEER_OFFSET,
        DrivebaseConstants.BR_STEER_ENCODER_REVERSED
    );

    private final Module BLSwerveModule = new Module(
        DrivebaseConstants.BL_DRIVE_MOTOR,
        DrivebaseConstants.BL_STEER_MOTOR,
        DrivebaseConstants.BL_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.BL_STEER_MOTOR_REVERSED,
        DrivebaseConstants.BL_STEER_ENCODER,
        DrivebaseConstants.BL_STEER_OFFSET,
        DrivebaseConstants.BL_STEER_ENCODER_REVERSED
    );
    
    private Gyro gyro = new Gyro();
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DrivebaseConstants.m_kinematics, getRotation2d());

    public SwerveSubsystem(){
       new Thread(() -> {
        try{
            Thread.sleep(1000);
            resetGyro();
        } catch(Exception e){
        } 
    }).start();
        
    
    }
    public void resetGyro(){
        gyro.setYaw(0);
    }
    public void zeroAllWhels(){
        FRSwerveModule.setDesiredState(new SwerveModuleState(0,new Rotation2d(0)));
        BRSwerveModule.setDesiredState(new SwerveModuleState(0,new Rotation2d(0)));
        FLSwerveModule.setDesiredState(new SwerveModuleState(0,new Rotation2d(0)));
        BLSwerveModule.setDesiredState(new SwerveModuleState(0,new Rotation2d(0)));
        
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(gyro.getHeading());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getRotation2d());
    }
    public void stopModules(){
        FRSwerveModule.stop();
        FLSwerveModule.stop();
        BRSwerveModule.stop();
        BLSwerveModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] ModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        FLSwerveModule.setDesiredState(ModuleStates[0]);
        FRSwerveModule.setDesiredState(ModuleStates[1]);;
        BLSwerveModule.setDesiredState(ModuleStates[2]);
        BRSwerveModule.setDesiredState(ModuleStates[3]);
        if(Constants.PrintDebugNumbers){
        SmartDashboard.putString("Module State FL", ModuleStates[0].toString());
        SmartDashboard.putString("Module State FR", ModuleStates[1].toString());
        SmartDashboard.putString("Module State BL", ModuleStates[2].toString());
        SmartDashboard.putString("Module State BR", ModuleStates[3].toString());

        SmartDashboard.putNumber("FL Voltage", FLSwerveModule.getDriveVoltage(ModuleStates[0]));
        SmartDashboard.putNumber("FR Voltage", FRSwerveModule.getDriveVoltage(ModuleStates[1]));
        SmartDashboard.putNumber("BL Voltage", BLSwerveModule.getDriveVoltage(ModuleStates[2]));
        SmartDashboard.putNumber("BR Voltage", BRSwerveModule.getDriveVoltage(ModuleStates[3]));
        }
    
    }

    public void printNumbers(){
        SmartDashboard.putNumber("FL Rad", FLSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR Rad", FRSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BL Rad", BLSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR Rad", BRSwerveModule.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("FL TRad", FLSwerveModule.getTurnignPosition());
        SmartDashboard.putNumber("FR TRad", FRSwerveModule.getTurnignPosition());
        SmartDashboard.putNumber("BL TRad", BLSwerveModule.getTurnignPosition());
        SmartDashboard.putNumber("BR TRad", BRSwerveModule.getTurnignPosition());

        }


    

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), FLSwerveModule.getState(), FRSwerveModule.getState(), BLSwerveModule.getState(), BRSwerveModule.getState());

        SmartDashboard.putNumber("Robot Heading", gyro.getHeading());
        SmartDashboard.putNumber("Robot Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Robot Roll", gyro.getRoll());

        SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());

        if(Constants.PrintDebugNumbers) {printNumbers(); SmartDashboard.delete("Print Debug Number?");}
        else{ 
            SmartDashboard.putBoolean("Print Debug Number?", false);
            Constants.PrintDebugNumbers = SmartDashboard.getBoolean("Print Debug Number?", false);
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

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   this.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                 DrivebaseConstants.m_kinematics, // SwerveDriveKinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }
     

}
