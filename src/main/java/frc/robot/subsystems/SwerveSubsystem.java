package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.Constants.drivebaseConstants.ModuleConstants;
import frc.robot.Constants.drivebaseConstants.deviceIDs;
import frc.robot.Constants.drivebaseConstants.kinematics;

public class SwerveSubsystem extends SubsystemBase{

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    private ShuffleboardLayout FrontLeft = tab.getLayout("Front Left", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
    private ShuffleboardLayout FrontRight = tab.getLayout("Front Right", BuiltInLayouts.kList).withPosition(2, 0).withPosition(2, 3);
    private ShuffleboardLayout BackLeft = tab.getLayout("Back Left", BuiltInLayouts.kList).withPosition(5, 0).withPosition(2, 3);
    private ShuffleboardLayout BackRight = tab.getLayout("Back Right", BuiltInLayouts.kList).withPosition(7, 0).withPosition(2, 3);

    

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

    private NetworkTableEntry FLVO = FrontLeft.add("Voltage", FLSwerveModule.getDriveVoltage()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    private NetworkTableEntry FLVE = FrontLeft.add("Velocity", FLSwerveModule.getDriveVelocity()).getEntry();
    private NetworkTableEntry FLTP = FrontLeft.add("Turning Pose", FLSwerveModule.getTurnignPosition()).getEntry();
    private NetworkTableEntry FLTV = FrontLeft.add("Turning Velocity", FLSwerveModule.getTurningVelocity()).getEntry();
    
    private NetworkTableEntry FRVO = FrontRight.add("Voltage", FRSwerveModule.getDriveVoltage()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    private NetworkTableEntry FRVE = FrontRight.add("Velocity", FRSwerveModule.getDriveVelocity()).getEntry();
    private NetworkTableEntry FRTP = FrontRight.add("Turning Pose", FRSwerveModule.getTurnignPosition()).getEntry();
    private NetworkTableEntry FRTV = FrontRight.add("Turning Velocity", FRSwerveModule.getTurningVelocity()).getEntry();

    private NetworkTableEntry BRVO = BackRight.add("Voltage", BRSwerveModule.getDriveVoltage()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    private NetworkTableEntry BRVE = BackRight.add("Velocity", BRSwerveModule.getDriveVelocity()).getEntry();
    private NetworkTableEntry BRTP = BackRight.add("Turning Pose", BRSwerveModule.getTurnignPosition()).getEntry();
    private NetworkTableEntry BRTV = BackRight.add("Turning Velocity", BRSwerveModule.getTurningVelocity()).getEntry();
    
    private NetworkTableEntry BLVO = BackLeft.add("Voltage", BLSwerveModule.getDriveVoltage()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    private NetworkTableEntry BLVE = BackLeft.add("Velocity", BLSwerveModule.getDriveVelocity()).getEntry();
    private NetworkTableEntry BLTP = BackLeft.add("Turning Pose", BLSwerveModule.getTurnignPosition()).getEntry();
    private NetworkTableEntry BLTV = BackLeft.add("Turning Velocity", BLSwerveModule.getTurningVelocity()).getEntry();

    

    private Gyro gyro = new Gyro();
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics.m_kinematics, getRotation2d());

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
    
    }

    
    public void printNumbers(){
        FLVO.setDouble(FLSwerveModule.getDriveVoltage());
        FLVE.setDouble(FLSwerveModule.getDriveVelocity());
        FLTP.setDouble(FLSwerveModule.getTurnignPosition());
        FLTV.setDouble(FLSwerveModule.getTurningVelocity());
        FRVO.setDouble(FRSwerveModule.getDriveVoltage());
        FRVE.setDouble(FRSwerveModule.getDriveVelocity());
        FRTP.setDouble(FRSwerveModule.getTurnignPosition());
        FRTV.setDouble(FRSwerveModule.getTurningVelocity());
        BRVO.setDouble(BRSwerveModule.getDriveVoltage());
        BRVE.setDouble(BRSwerveModule.getDriveVelocity());
        BRTP.setDouble(BRSwerveModule.getTurnignPosition());
        BRTV.setDouble(BRSwerveModule.getTurningVelocity());
        BLVO.setDouble(BLSwerveModule.getDriveVoltage());
        BLVE.setDouble(BLSwerveModule.getDriveVelocity());
        BLTP.setDouble(BLSwerveModule.getTurnignPosition());
        BLTV.setDouble(BLSwerveModule.getTurningVelocity());
        }


    @Override
    public void periodic() {
        odometry.update(getRotation2d(), FLSwerveModule.getState(), FRSwerveModule.getState(), BLSwerveModule.getState(), BRSwerveModule.getState());

        SmartDashboard.putNumber("Robot Heading", gyro.getHeading());
        SmartDashboard.putNumber("Robot Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Robot Roll", gyro.getRoll());

        SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());


        if(IO.PrintDebugNumbers) {printNumbers(); SmartDashboard.delete("Print Debug Number?");}
        else{ 
            SmartDashboard.putBoolean("Print Debug Number ", false);
            IO.PrintDebugNumbers = SmartDashboard.getBoolean("Print Debug Number?", false);
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
                 kinematics.m_kinematics, // SwerveDriveKinematics
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
