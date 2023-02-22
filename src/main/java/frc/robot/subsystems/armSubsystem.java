package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.armAndIntakeConstants;
import frc.robot.Constants.armAndIntakeConstants.armConstants;;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private PIDController ArmPID = new PIDController(0.1, 0, 0);
    

    

    
    public armSubsystem(){
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);

        
        ArticulateL.getEncoder().setPositionConversionFactor(1/182 * 360 * 2);
        ArticulateR.getEncoder().setPositionConversionFactor((1/182) * (1/Math.PI) * 180);

        setMotorPosition(0, 0);
        //Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);
        
        /* Articulate motor gear ratios */
        // 1 : 20 
        // 24 : 78 
        // 15 : 42 
        // 360 : 65520
        // 1 : 182
        /*                               */


        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);
        //Extend.setInverted(false);
        
        tab.add("Left Motor Voltage", ArticulateL.get());
        tab.add("Left Motor Pose", ArticulateR.getEncoder().getPosition());
        tab.add("Right Motor Voltage", ArticulateR.get());
        tab.add("Right Motor Pose", ArticulateL.getEncoder().getPosition());
        
    }

    public void moveToPos(double pose){
        SimpleMotorFeedforward ArmFeedforward = new SimpleMotorFeedforward(armConstants.kS, armConstants.kV, armConstants.kA);
        //FIXME Need to characterize Arm
        
        ManualJob(ArmPID.calculate(AvgPose() - pose));
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

    public double rightMotorPosition(){
        return ArticulateR.getEncoder().getPosition();
    }

    public double leftMotorPosition(){
        return ArticulateR.getEncoder().getPosition();
    }

    public double AvgPose(){
        return (leftMotorPosition() + rightMotorPosition())/2;
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

    }
    
    

    @Override
    public void periodic() {
        if(IO.PrintDebugNumbers){
            SmartDashboard.putNumber("Right Pose", rightMotorPosition());
            SmartDashboard.putNumber("Left Pose", leftMotorPosition());

        }
    }
    
}
