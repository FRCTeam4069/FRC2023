package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.armAndIntakeConstants.armConstants;;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    public RelativeEncoder leftEncoder, rightEncoder, extendEncoder;
    public double joystickValues;
    

    

    
    public armSubsystem(){
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        ArticulateL.getEncoder().setPositionConversionFactor(2);
        ArticulateR.getEncoder().setPositionConversionFactor(2);
        Extend.getEncoder().setPositionConversionFactor((1/392) * 0.5);

        setZero();

        ArticulateL.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);
        
        ArticulateL.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kReverse, true);

        
        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);

        Extend.setInverted(armConstants.telescopeMotorInvert);
        
        
    }

    public boolean moveToPos(double pose, double maxSpeed) {  
        if(Math.abs(pose) > 140) {return false;}
        else{
        if(maxSpeed < 0){ throw new IllegalArgumentException("maxSpeed must be 0 to 1"); }
        
        
        double speed = 
        MathUtil.clamp( 
            (
              (pose - AvgPose())*armConstants.proportionalGain 
            + (ExtendedPose()*AvgPose()*armConstants.GravGain*getSide()) ),
            
            -maxSpeed, maxSpeed);
        
        manualArticulate(speed);

        return true;
        }

    }

    public void manualArticulate(double speed){
        ArticulateL.set(speed);
        ArticulateR.set(speed);
    }

    public void manualExtend(double speed){
        Extend.set(speed);
    }

    public void stop(){
        ArticulateL.stopMotor();
        ArticulateR.stopMotor();
    }

    public double rightMotorSpeed(){
        return ArticulateR.getEncoder().getVelocity();
    }

    public double leftMotorSpeed(){
        return ArticulateL.getEncoder().getVelocity();
    }

    public double rightMotorPosition(){
        return ArticulateR.getEncoder().getPosition();
    }

    public double leftMotorPosition(){
        return ArticulateR.getEncoder().getPosition();
    }

    public double AvgPose(){
        return (leftMotorPosition() + rightMotorPosition())/2;
    }

    public double ExtendedPose(){
        return Extend.getEncoder().getPosition();
    }

    public void setMotorPosition(double Right, double Left){
        ArticulateL.getEncoder().setPosition(Left);
        ArticulateR.getEncoder().setPosition(Right);
    }

    public void setZero(){
        setMotorPosition(0, 0);
        joystickValues =0;
        Extend.getEncoder().setPosition(0);

    }

    /**
     * @return +ve or -ve 1 depeneding on which side the arm is on
     */
    public double getSide(){
        return Math.copySign(1, AvgPose());
    }

    public void joystickValues(double joystick){
        joystickValues += joystick;
    }

    /** 
     * Extend to position and velocity
     * @param Position Position to extend to (in)
     * @param maxSpeed 0 to 1
     */
    public void extentToPosition(double Position, double maxSpeed){
        if(maxSpeed < 0){ throw new IllegalArgumentException("maxSpeed must be 0 to 1"); }

        double speed = MathUtil.clamp( (Position - ExtendedPose())*0.1, -maxSpeed, maxSpeed);
        manualExtend(speed);

    }
    
    /**
     * Set the home position for the extending motor
     */
    public void setHome(){
        Extend.getEncoder().setPosition(0);
    }
    
    

    @Override
    public void periodic() {
        if(IO.PrintDebugNumbers){
            SmartDashboard.putNumber("Right Pose", rightMotorPosition());
            SmartDashboard.putNumber("Left Pose", leftMotorPosition());
            SmartDashboard.putNumber("Number of Ticks Extend", Extend.getEncoder().getCountsPerRevolution());
            SmartDashboard.putNumber("Number of Ticks Ar", ArticulateL.getEncoder().getCountsPerRevolution());
            SmartDashboard.putNumber("Number of Ticks Al", ArticulateR.getEncoder().getCountsPerRevolution());
            SmartDashboard.putNumber("Joystick Positions", joystickValues);
        }
        SmartDashboard.putNumber("Lead Screw Rotations: ", ExtendedPose());

    }
    
}
