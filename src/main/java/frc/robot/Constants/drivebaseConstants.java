// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class drivebaseConstants {

    public static final class SparkMaxVoltage{
        public static final int DriveingMotorVoltage = 20;
        public static final int TurningMotorVoltage = 20;
    }

    public static final class kinematics{

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(9.375*2); 
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(9.375*2); 

        public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );   
    }

    public static final class ModuleConstants{
    public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = SdsModuleConfigurations.MK4I_L2.getDriveReduction() ;
    public static final double kTurningMotorGearRatio = SdsModuleConfigurations.MK4I_L2.getSteerReduction();
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
    }

    public static class CharacterizationData{

        public static final double Ks = 0.63253;
        public static final double Kv = 2.2936;
        public static final double Ka = 0.18409;

        public static final double Kp = 2.6902;
        public static final double Kd = 0;

        public static final SimpleMotorFeedforward feedForwardController = new SimpleMotorFeedforward(Ks, Kv, Ka);

    }

    public static final class deviceIDs{


        public static final int PIGEON_ID = 0;

        public static final int FL_DRIVE_MOTOR = 16;
        public static final int FL_STEER_MOTOR = 17; 
        public static final boolean FL_DRIVE_MOTOR_REVERSED = true;                        
        public static final boolean FL_STEER_MOTOR_REVERSED = true;                       
        public static final int FL_STEER_ENCODER = 10; 
        public static final double FL_STEER_OFFSET = -3.110 + Math.PI;// -3.149262557529221  + (Math.PI/2);  
        public static final boolean FL_STEER_ENCODER_REVERSED = !FL_STEER_MOTOR_REVERSED;                      
    
        public static final int FR_DRIVE_MOTOR = 2; 
        public static final int FR_STEER_MOTOR = 3;
        public static final boolean FR_DRIVE_MOTOR_REVERSED = true;                        
        public static final boolean FR_STEER_MOTOR_REVERSED = true;                      
        public static final int FR_STEER_ENCODER = 9; 
        public static final double FR_STEER_OFFSET = -3.129+ Math.PI;//-3.149262557529221  - (3*Math.PI/2);
        public static final boolean FR_STEER_ENCODER_REVERSED = !FR_STEER_MOTOR_REVERSED;                      
    
        public static final int BL_DRIVE_MOTOR = 18; 
        public static final int BL_STEER_MOTOR = 19; 
        public static final boolean BL_DRIVE_MOTOR_REVERSED = true;                        
        public static final boolean BL_STEER_MOTOR_REVERSED = true;                        
        public static final int BL_STEER_ENCODER = 6;
        public static final double BL_STEER_OFFSET = -3.124+ Math.PI;// -3.158466442256535  + (Math.PI/2);
        public static final boolean BL_STEER_ENCODER_REVERSED = !BL_STEER_MOTOR_REVERSED;                     
    
        public static final int BR_DRIVE_MOTOR = 20; 
        public static final int BR_STEER_MOTOR = 1; 
        public static final boolean BR_DRIVE_MOTOR_REVERSED = true;                        
        public static final boolean BR_STEER_MOTOR_REVERSED = true;                       
        public static final int BR_STEER_ENCODER = 11;         
        public static final double BR_STEER_OFFSET = -(3.62 + Math.PI/2)+ Math.PI;//-1.107534128853433  - (3*Math.PI/2);
        public static final boolean BR_STEER_ENCODER_REVERSED = !BR_STEER_MOTOR_REVERSED;                      
    }

}

