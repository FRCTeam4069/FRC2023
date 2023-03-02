package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.Constants.drivebaseConstants.CharacterizationData;
import frc.robot.Constants.drivebaseConstants.ModuleConstants;
import frc.robot.Constants.drivebaseConstants.deviceIDs;
import frc.robot.Constants.drivebaseConstants.kinematics;

public class SwerveSubsystem extends SubsystemBase{
    

    public final Module FRSwerveModule = new Module(
        deviceIDs.FR_DRIVE_MOTOR,
        deviceIDs.FR_STEER_MOTOR,
        deviceIDs.FR_DRIVE_MOTOR_REVERSED, 
        deviceIDs.FR_STEER_MOTOR_REVERSED,
        deviceIDs.FR_STEER_ENCODER,
        deviceIDs.FR_STEER_OFFSET,
        deviceIDs.FR_STEER_ENCODER_REVERSED
    );

    private final Module FLSwerveModule = new Module(
        deviceIDs.FL_DRIVE_MOTOR,
        deviceIDs.FL_STEER_MOTOR,
        deviceIDs.FL_DRIVE_MOTOR_REVERSED, 
        deviceIDs.FL_STEER_MOTOR_REVERSED,
        deviceIDs.FL_STEER_ENCODER,
        deviceIDs.FL_STEER_OFFSET,
        deviceIDs.FL_STEER_ENCODER_REVERSED
    );

    private final Module BRSwerveModule = new Module(
        deviceIDs.BR_DRIVE_MOTOR,
        deviceIDs.BR_STEER_MOTOR,
        deviceIDs.BR_DRIVE_MOTOR_REVERSED, 
        deviceIDs.BR_STEER_MOTOR_REVERSED,
        deviceIDs.BR_STEER_ENCODER,
        deviceIDs.BR_STEER_OFFSET,
        deviceIDs.BR_STEER_ENCODER_REVERSED
    );

    private final Module BLSwerveModule = new Module(
        deviceIDs.BL_DRIVE_MOTOR,
        deviceIDs.BL_STEER_MOTOR,
        deviceIDs.BL_DRIVE_MOTOR_REVERSED, 
        deviceIDs.BL_STEER_MOTOR_REVERSED,
        deviceIDs.BL_STEER_ENCODER,
        deviceIDs.BL_STEER_OFFSET,
        deviceIDs.BL_STEER_ENCODER_REVERSED

        
    );



    public static final Gyro gyro = new Gyro(deviceIDs.PIGEON_ID);
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics.m_kinematics, getRotation2d(), new SwerveModulePosition[]{
        FRSwerveModule.getPosition(),
        FLSwerveModule.getPosition(),
        BRSwerveModule.getPosition(),
        BLSwerveModule.getPosition(),
    });
    public SwerveSubsystem(){
       new Thread(() -> {
        try{
            Thread.sleep(1000);
            resetGyro();
        } catch(Exception e){
        } 
    }).start();
    
  
    Shuffleboard.update();
    
    }
    public void resetGyro(){
        gyro.setYaw(90);
    }
    public Gyro getGyro(){
        return gyro;
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
        odometry.resetPosition(getRotation2d(),new SwerveModulePosition[]{
            FRSwerveModule.getPosition(),
            FLSwerveModule.getPosition(),
            BRSwerveModule.getPosition(),
            BLSwerveModule.getPosition(),
        } ,  pose);
    }
    public void stopModules(){
        FRSwerveModule.stop();
        FLSwerveModule.stop();
        BRSwerveModule.stop();
        BLSwerveModule.stop();
    }

    public CommandBase resetGyroCommmand(){
        return this.runOnce(() -> resetGyro());
    }

    public void setModuleStates(SwerveModuleState[] ModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        FLSwerveModule.setDesiredState(ModuleStates[0]);
        FRSwerveModule.setDesiredState(ModuleStates[1]);;
        BLSwerveModule.setDesiredState(ModuleStates[2]);
        BRSwerveModule.setDesiredState(ModuleStates[3]);
    
    }

    
   


    @Override
    public void periodic() {
        odometry.update(getRotation2d(),new SwerveModulePosition[]{
            FRSwerveModule.getPosition(),
            FLSwerveModule.getPosition(),
            BRSwerveModule.getPosition(),
            BLSwerveModule.getPosition()});

        SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());
            System.out.println("GYROLL" +  gyro.getRoll());


       

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
                 kinematics.m_kinematics, // SwerveDriveKinematics
                 CharacterizationData.autoXController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 CharacterizationData.autoYController, // Y controller (usually the same values as X controller)
                 CharacterizationData.autoThetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }
     

}
