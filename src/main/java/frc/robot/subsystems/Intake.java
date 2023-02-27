package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class Intake extends SubsystemBase{
    private CANSparkMax intake, wrist;
    private RelativeEncoder intakeEncoder, wristEncoder;
 

    public Intake(){
    

        intake= new CANSparkMax(intakeConstants.INTAKE_ID , MotorType.kBrushless);
        wrist= new CANSparkMax(intakeConstants.WRIST_ID , MotorType.kBrushless);


        intake.setSmartCurrentLimit(30);
        wrist.setSmartCurrentLimit(40);
        wristEncoder = wrist.getEncoder();
        intakeEncoder = intake.getEncoder();

        wrist.getOutputCurrent();
        /*
         * (180 - Armangle) % 90 -> lock wirst to be parallel to ground 
         * ((-Armangle - 90) %90) -> lock wirst to be prependicular to ground 
         */

         /*
          * 3 : 1, 60 : 24
          * 7.5 : 1 
          */


    }

    public boolean WristCurrentSpike(){
        return false;
    }
    public boolean IntakeCurrentSpike(){
        return false;
    }

    /**
     * Set the speed of the wrist
     * @param speed -iv is down and +iv is up
     */
    public void setWrist(double speed){
        wrist.set(speed);
    }

    /**
     * Set the speed of the intake
     * @param speed -iv is inward and +iv is outward
     */
    public void setIntake(double speed){
        intake.set(speed);
    }

    /**
     * @return the current set speed of the wrist motor
     */
    public double getWrist(){
        return wrist.get();
    }

    /**
     * @return the current set speed of the intake motor
     */
    public double getIntake(){
        return intake.get();
    }

    /**
     * @return the current position of the wrist encoder
     */
    public double getWristPose(){
        return wristEncoder.getPosition();
    }


    /**
     * @return the current position of the intake encoder
     */
    public double getIntakePose(){
        return intakeEncoder.getPosition();
    }

    /**
     * set position of the wrist encoder
     * @param pose New position of the wrist encoder
     */
    public void setWristPose(double pose){
        wristEncoder.setPosition(pose);
    }


    /**
     * set position of the intake encoder
     * @param pose New position of the intake encoder
     */
    public void setIntakePose(double pose){
        intakeEncoder.setPosition(pose);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}


