package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private Encoder encoder;
    private DigitalInput limitLeft, limitRight;
    private double power; double speed; 
    private double torque = 0, requestedTorque = 0;

    public final int[] requiedTorque = {10, 20}; /**not exact values**/
    public Intake(){
        intakeMotor = new CANSparkMax(intakeConstants.INTAKE_ID , MotorType.kBrushless);
        encoder = new Encoder(intakeConstants.ENCODER_ID_A , intakeConstants.ENCODER_ID_B);
        limitLeft = new DigitalInput(intakeConstants.LIMIT_SWITCH_ID_1);
        limitRight = new DigitalInput(intakeConstants.LIMIT_SWITCH_ID_2);

        power = 0;
        torque = 0;
    }

    public void update(Use d, boolean Cone){
        requestedTorque = Cone ? requiedTorque[0] : requiedTorque[1];

        intakeMotor.set(0.5);
        power = 0.5;

        torque = (9.5488 * power) / encoder.getRate();

        SmartDashboard.putNumber("Intake Torque", torque);
    }

    public enum Use{ //might be helpful
       IN,
       OUT
    }    
}


