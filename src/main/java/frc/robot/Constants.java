// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalSource;

public final class Constants {

    public static boolean PrintDebugNumbers = true; // prints Debug info for wheels and speeds 
    public static boolean enableSlewrateLimiter = true;
    public static boolean limelightPluggedIn = false;

    public static final class SparkMaxVoltage{
        public static final int DriveingMotorVoltage = 20;
        public static final int TurningMotorVoltage = 20;
    }

    public static final class DrivebaseConstants{

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(9.375*2); 
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(9.375*2); 
    
        public static final int PIGEON_ID = 20;

        public static final int FL_DRIVE_MOTOR = 16;
        public static final int FL_STEER_MOTOR = 17; 
        public static final boolean FL_DRIVE_MOTOR_REVERSED = true;                        //FIXME
        public static final boolean FL_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int FL_STEER_ENCODER = 10; 
        public static final double FL_STEER_OFFSET =  -3.149262557529221;//(-4.7139) - (-4.71699) ; //-2         //FIXME
        public static final boolean FL_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int FR_DRIVE_MOTOR = 2; 
        public static final int FR_STEER_MOTOR = 3;
        public static final boolean FR_DRIVE_MOTOR_REVERSED = true;                        //FIXME
        public static final boolean FR_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int FR_STEER_ENCODER = 9; 
        public static final double FR_STEER_OFFSET = -3.149262557529221;
        public static final boolean FR_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int BL_DRIVE_MOTOR = 18; 
        public static final int BL_STEER_MOTOR = 19; 
        public static final boolean BL_DRIVE_MOTOR_REVERSED = true;                        //FIXME
        public static final boolean BL_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int BL_STEER_ENCODER = 12;
        public static final double BL_STEER_OFFSET = -3.158466442256535;
        public static final boolean BL_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int BR_DRIVE_MOTOR = 0; 
        public static final int BR_STEER_MOTOR = 1; 
        public static final boolean BR_DRIVE_MOTOR_REVERSED = true;                        //FIXME
        public static final boolean BR_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int BR_STEER_ENCODER = 11;         
        public static final double BR_STEER_OFFSET = -1.107534128853433;
        public static final boolean BR_STEER_ENCODER_REVERSED = true;                      //FIXME

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
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    public static final double kMaxAngularSpeedRadiansPerSecond = //
    2*2*Math.PI / 10;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
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

    public static final class InputConstants{

        public static final double Contoller1portNum = 0;
        public static final double maxTurnSpeed = 3; // Meters/sec
        public static final double maxSpeed = 5; // Meters/sec

        public static final double xdeadZone = 0.05;
        public static final double ydeadZone = 0.05;
        public static final double tdeadZone = 0.05;

        public static final double xSpeedSlewRate = 5;
        public static final double ySpeedSlewRate = 5;
        public static final double TurnSpeedSlewRate = 3;

    }

    public static class CharacterizationData{

        public static final double Ks = 0.63253;
        public static final double Kv = 2.2936;
        public static final double Ka = 0.18409;

        public static final double Kp = 2.6902;
        public static final double Kd = 0;

        public static final SimpleMotorFeedforward feedForwardController = new SimpleMotorFeedforward(Ks, Kv);
    }

    public static class cameraInfo{


    }


    public static class IntakeConstants{

        public static final int INTAKE_ID = 0;
        public static final DigitalSource ENCODER_ID_A = null;
        public static final int LIMIT_SWITCH_ID_2 = 0;
        public static final int LIMIT_SWITCH_ID_1 = 0;
        public static final DigitalSource ENCODER_ID_B = null;

        

    }
    public static final byte INTAKE_ID = 0;
    
    public static final int ARM_ID_L = 15; // left
    public static final int ARM_ID_R = 4; // right
    public static final int ARM_ID_E = 0; // Extend //FIXME

}
