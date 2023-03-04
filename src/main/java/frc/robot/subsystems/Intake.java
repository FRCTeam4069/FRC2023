package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax intake, wrist;
    private RelativeEncoder intakeEncoder, wristEncoder;
    public int controlType;
    
    public boolean enableLimit;

    public Intake() {
        controlType = 0;

        intake = new CANSparkMax(intakeConstants.INTAKE_ID, MotorType.kBrushless);
        wrist = new CANSparkMax(intakeConstants.WRIST_ID, MotorType.kBrushless);

        intake.setSmartCurrentLimit(40);
        wrist.setSmartCurrentLimit(30);


        wristEncoder = wrist.getEncoder();
        intakeEncoder = intake.getEncoder();

        wrist.getOutputCurrent();
        /*
         * wrist is 70:1
         * meaning 2940 encoder ticks : 1 wrist rotation
         * 1/2940
         */

         wristEncoder.setPositionConversionFactor(47/70);

        setWristPose(0);
   //     setIntakePose(16.8 - 4.3);
        setIntakePose(0);
  
          intake.setSoftLimit(SoftLimitDirection.kReverse, 0);
        intake.setSoftLimit(SoftLimitDirection.kForward, 16);





    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("wrist Pose", getWristPose());
        SmartDashboard.putNumber("grabber pose", getIntakePose());

        intake.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        intake.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);

    }
    
    public void wristToPose(double position){
        setWrist((position - getWristPose()*47) * intakeConstants.wristkP);
    }

    /**
     *
     * @param mode 1 : Manual controll, 2 : Parallel Lock, 3 : Perpendicular Lock
     * @return command to mode
     */
    public CommandBase setMode(int mode) {
        return this.runOnce(() -> controlType = mode);
    }

    /**
     * Set the speed of the wrist
     * 
     * @param speed -iv is down and +iv is up
     */
    public void setWrist(double speed) {
        wrist.set(speed);
    }

    /**
     * Set the speed of the intake
     * 
     * @param speed -iv is inward and +iv is outward
     */
    public void setIntake(double speed) {
        intake.set(speed);
    }

    /**
     * @return the current set speed of the wrist motor
     */
    public double getWrist() {
        return wrist.get();
    }

    /**
     * @return the current set speed of the intake motor
     */
    public double getIntake() {
        return intake.get();
    }

    /**
     * @return the current position of the wrist encoder
     */
    public double getWristPose() {
        return wristEncoder.getPosition();
    }

    /**
     * @return the current position of the intake encoder
     */
    public double getIntakePose() {
        return intakeEncoder.getPosition();
    }

    /**
     * set position of the wrist encoder
     * 
     * @param pose New position of the wrist encoder
     */
    public void setWristPose(double pose) {
        wristEncoder.setPosition(pose);
    }

    /**
     * set position of the intake encoder
     * 
     * @param pose New position of the intake encoder
     */
    public void setIntakePose(double pose) {
        intakeEncoder.setPosition(pose);
    }

    public CommandBase zeroWrist(double pose) {
        return this.runOnce(() -> setWristPose(pose));
    }
    public CommandBase zeroIntake(double pose) {
        return this.runOnce(() -> setIntakePose(pose));
    }

    public CommandBase diableLimit(){
        return this.runOnce(() -> enableLimit=false);
    }  
    public CommandBase enableLimit(){
        return this.runOnce(() -> enableLimit=true);
    }

  

}
