package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    
    private Pigeon2 gyro = new Pigeon2(DrivebaseConstants.PIGEON_ID);


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

    public double getHeading(){
        return gyro.getYaw() % 360;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
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

        SmartDashboard.putString("Module State FL", ModuleStates[0].toString());
        SmartDashboard.putString("Module State FR", ModuleStates[1].toString());
        SmartDashboard.putString("Module State BL", ModuleStates[2].toString());
        SmartDashboard.putString("Module State BR", ModuleStates[3].toString());

        SmartDashboard.putNumber("FL Voltage", FLSwerveModule.getDriveVoltage(ModuleStates[0]));
        SmartDashboard.putNumber("FR Voltage", FRSwerveModule.getDriveVoltage(ModuleStates[1]));
        SmartDashboard.putNumber("BL Voltage", BLSwerveModule.getDriveVoltage(ModuleStates[2]));
        SmartDashboard.putNumber("BR Voltage", BRSwerveModule.getDriveVoltage(ModuleStates[3]));
    
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
        SmartDashboard.putNumber("Robot Heading", getHeading());
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

}
