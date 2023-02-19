package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;

    
    public armSubsystem(){
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL  = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        //Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);

        //Extend.setInverted(false);
    }

    public void moveToPos(int posCode){

    }

    public void ManualJob(double speed){
        ArticulateL.set(speed);
        ArticulateR.set(speed);
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
    
    

    @Override
    public void periodic() {
    }
    
}
