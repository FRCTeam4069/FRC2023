package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.armAndIntakeConstants.armConstants;;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    

    

    
    public armSubsystem(){
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        
        ArticulateL.getEncoder().setPositionConversionFactor(2);//(1/182) * (ArticulateL.getEncoder().getCountsPerRevolution()));
        ArticulateR.getEncoder().setPositionConversionFactor(2);//(1/182) * (ArticulateL.getEncoder().getCountsPerRevolution()));
        Extend.getEncoder().setPositionConversionFactor((1/392) * 0.5);

        setMotorPosition(0, 0);
        
        
        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);
        Extend.setInverted(armConstants.telescopeMotorInvert);
        
        
    }

    public void moveToPos(double pose, double maxSpeed) {  
        double speed = MathUtil.clamp( (pose - AvgPose())*0.1, -maxSpeed, maxSpeed);
        manualArticulate(speed);
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
            SmartDashboard.putNumber("Lead Screw Rotations: ", ExtendedPose());
            SmartDashboard.putNumber("Number of Ticks Extend", Extend.getEncoder().getCountsPerRevolution());
            SmartDashboard.putNumber("Number of Ticks Ar", ArticulateL.getEncoder().getCountsPerRevolution());
            SmartDashboard.putNumber("Number of Ticks Al", ArticulateR.getEncoder().getCountsPerRevolution());

        }
    }
    
}
