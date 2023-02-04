package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax Articulate, Extend;

    private int[][] RequiredEncoderPulses = {

        {1000, 1000},

        {2000, 2000},

        {3000, 3000} //first number is swish swish, second is extendo

    };
    
    public Arm(){
        Articulate = new CANSparkMax(Constants.ARM_ID_1, MotorType.kBrushless);
        Extend = new CANSparkMax(Constants.ARM_ID_2, MotorType.kBrushless);
    }

    public void moveToPos(int posCode){

    }

    public void ManualJob(double speed){

    }
}
