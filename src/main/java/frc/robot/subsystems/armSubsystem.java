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

    public void moveToPos(double pose) {  
        double speed = MathUtil.clamp( (AvgPose() - Math.abs(pose))*0.1, -0.5, 0.5);
        manualArticulate(Math.copySign(speed, pose));
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
     * @param Position Position to extend to (cm)
     * @param Speed 0 to 1
     */
    public void extentToPosition(double Position, double velocity){}
    
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
