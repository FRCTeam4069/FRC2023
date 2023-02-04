package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    CANSparkMax intakeMotor;

    public Intake(){
        intakeMotor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
    }

    public void update(Direction d, boolean withCameraInput){

    }

    public enum Direction{ //might be helpful
        IN_CONE,
        OUT_CONE,
        IN_CUBE,
        OUT_CUBE
    }    
}


