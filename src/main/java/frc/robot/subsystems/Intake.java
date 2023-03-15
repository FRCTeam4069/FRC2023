package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax intake, wrist;
    private RelativeEncoder intakeEncoder, wristEncoder;
    public double side, wristTarget, low, gravGain, armAngle;    
    public boolean enableLimit = true, _enableLimit = false;

    public Intake() {

        intake = new CANSparkMax(intakeConstants.INTAKE_ID, MotorType.kBrushless);
        wrist = new CANSparkMax(intakeConstants.WRIST_ID, MotorType.kBrushless);

        intake.setSmartCurrentLimit(40);
        wrist.setSmartCurrentLimit(40);
        wrist.setInverted(true);
        wrist.setOpenLoopRampRate(0);

        wristEncoder = wrist.getEncoder();
        intakeEncoder = intake.getEncoder();

        /*
         * wrist is 70:1
         * meaning 2940 encoder ticks : 1 wrist rotation
         * 1/2940
         */

         wristEncoder.setPositionConversionFactor(1);
         wristEncoder.setVelocityConversionFactor(1);
         SmartDashboard.putNumber("wrist", wristEncoder.getPositionConversionFactor());

        setWristPose(-40);
        intakeEncoder.setPosition(16.8 - 4.3);

        //setIntakePose(0);
        intake.setSoftLimit(SoftLimitDirection.kReverse, 0);
        intake.setSoftLimit(SoftLimitDirection.kForward, 16);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, -40);
        wrist.setSoftLimit(SoftLimitDirection.kForward, 39);
        

    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        intakeConstants.wristPose = getWristAngle();
        side = armConstants.side;
        armAngle = armConstants.armPose;
        if(enableLimit != _enableLimit){
        intake.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        intake.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
        wrist.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        wrist.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
        _enableLimit = enableLimit;
        }
        SmartDashboard.putNumber("Wrist Subsystem received Arm side", side);
        SmartDashboard.putNumber("wrist angle", getWristAngle());
        SmartDashboard.putNumber("wrist Target (Subsystem)", wristTarget);
        low = (180 - Math.abs(armConstants.armPose)) * armConstants.side;
        SmartDashboard.putNumber("Low", low);
        SmartDashboard.putNumber("gravGain t1", ((getWristAngle() - low)*0.01));
        SmartDashboard.putNumber("gravGain t2 (sin)", Math.sin(Math.toRadians(getWristAngle() - low)) * 0.1);// * (0 - getWristAngle()));
        SmartDashboard.putNumber("gravGain t3 (cos)", Math.cos(Math.toRadians(getWristAngle() - low)));
        SmartDashboard.putNumber("parallel angle", low - (side*90));

        

    }
    
    // +iv is up in negitive Side (arm)
    // -iv is up in positive Side (arm)

    public double getWristAngle(){
        return (getWristPose()*2/80 * 120);
    }

    public void intakeToPose(double position){
        setIntake((position - getIntakePose()) * intakeConstants.intakekP);
    }

    public void setWrist(double speed) {
        wrist.set(speed);
    }
    public void setIntake(double speed) {
        intake.set(speed);
    }


    public double getWristPose() {
        return wristEncoder.getPosition();
    }
    public double getIntakePose() {
        return intakeEncoder.getPosition();
    }

    public void setWristPose(double pose) {
        wristEncoder.setPosition(pose);
    }  
    public void setIntakePose(double pose) {
        intakeEncoder.setPosition(pose);
    }

    public CommandBase zeroWrist(double pose) {
        return this.runOnce(() -> setWristPose(pose));
    }
    public CommandBase zeroIntake(double pose) {
        return this.runOnce(() -> intakeEncoder.setPosition(pose));
    }

    public CommandBase diableLimit(){
        return this.runOnce(() -> enableLimit=false);
    }  
    public CommandBase enableLimit(){
        return this.runOnce(() -> enableLimit=true);
    }

  

}
