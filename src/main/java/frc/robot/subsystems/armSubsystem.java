package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;


public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    public RelativeEncoder leftEncoder, rightEncoder, extendEncoder;
    public boolean enableLimit;
    public double extendPose = 0, articulatePose = 0;
    public int articulateLimit = 130;
    public boolean moveToPoseMode = true;
    

    public armSubsystem() {
        enableLimit = true;
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        Extend.setInverted(false);

        ArticulateR.getEncoder().setPositionConversionFactor(2);
        Extend.getEncoder().setPositionConversionFactor(0.5/2.33334);

        ArticulateL.burnFlash();
        ArticulateR.burnFlash();
        Extend.burnFlash();

        ArticulateL.getEncoder().setVelocityConversionFactor(1);
        ArticulateR.getEncoder().setVelocityConversionFactor(1);

        armConstants.side = getSide();
        setZero();
        ArticulateL.setSoftLimit(SoftLimitDirection.kForward, articulateLimit);
        ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -articulateLimit);
        ArticulateR.setSoftLimit(SoftLimitDirection.kForward, articulateLimit);
        ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -articulateLimit);

        ArticulateL.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kReverse, true);

        Extend.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Extend.setSoftLimit(SoftLimitDirection.kForward, 33);
        Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);

        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);

        Extend.setInverted(armConstants.telescopeMotorInvert);

        ArticulateL.setOpenLoopRampRate(0);
        ArticulateR.setOpenLoopRampRate(0);

      


        SmartDashboard.putNumber("Articulate", ArticulateL.getEncoder().getCountsPerRevolution());
        SmartDashboard.putNumber("Extend", Extend.getEncoder().getCountsPerRevolution());
        SmartDashboard.putNumber("Articulate Multi", ArticulateL.getEncoder().getPositionConversionFactor());
        SmartDashboard.putNumber("Extend Multi", Extend.getEncoder().getPositionConversionFactor());
    }

    @Override
    public void periodic() {
        armConstants.side = getSide();
        armConstants.ArmPose = AvgPose();

        Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
        ArticulateL.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);

        SmartDashboard.putNumber("Arm: Articulate", rightMotorPosition());
        SmartDashboard.putNumber("Arm: Target Articulate", articulatePose);
        SmartDashboard.putNumber("Arm: Extend", ExtendedPose());


        if(enableLimit){
        extendPose = MathUtil.clamp(extendPose, 0, 33);
        articulatePose = MathUtil.clamp(articulatePose, -articulateLimit , articulateLimit);}

        if(moveToPoseMode){moveToPose(articulatePose);}
        //extendToPose(extendPose, 1);
    
        
    }

    private boolean moveToPose(double pose) {

        double speed = MathUtil.clamp(
                ((pose - AvgPose()) * armConstants.proportionalGain
                        + (ExtendedPose() * getSide() * armConstants.GravGain)),

                -1, 1);

        manualArticulate(speed);

        return true;

    }

    public void extendToPose(double Position, double maxSpeed) {
        double speed = MathUtil.clamp((Position - ExtendedPose()) * 1, -maxSpeed, maxSpeed);
        manualExtend(speed);

    }

    public void manualArticulate(double speed) {

        ArticulateL.set(speed);
        ArticulateR.set(speed);

        if(!(intakeConstants.wristPose * getSide() < 0) ){
        articulateLimit = 130;  
        } 
        else{ 
            articulateLimit = 110;
        }
    }

    public void manualExtend(double speed) {
        Extend.set(speed);
    }

    public void setArmPose(double pose) {
        this.articulatePose = pose;
    }

    public void setExtendPose(double pose) {
        this.extendPose = pose;
    }

    public boolean isAtPoseAT(double Pose, double threshold) {
        return (Math.abs(Pose - AvgPose()) < threshold);

    }

    public boolean isAtPoseE(double Pose, double threshold) {
        return (Math.abs(Pose - extendPose) < threshold);
    }

    public void stop() {
        ArticulateL.stopMotor();
        ArticulateR.stopMotor();
        Extend.stopMotor();
    }

    public double rightMotorPosition() {
        return ArticulateR.getEncoder().getPosition();
    }

    public double leftMotorPosition() {
        return ArticulateL.getEncoder().getPosition();
    }

    public double AvgPose() {
        return ((ArticulateR.getEncoder().getPosition() + ArticulateR.getEncoder().getPosition()) / 2);
    }

    public double ExtendedPose() {
        return Extend.getEncoder().getPosition();
    }

    public void setMotorPosition(double Right, double Left) {
        ArticulateL.getEncoder().setPosition(Left);
        ArticulateR.getEncoder().setPosition(Right);
    }

    public void setZero() {
        setMotorPosition(0, 0);
        Extend.getEncoder().setPosition(0);
        articulatePose = 0;
    }

    /**
     * @return +ve or -ve 1 depeneding on which side the arm is on
     */
    public double getSide() {
        return Math.copySign(1, AvgPose());
    }

    /**
     * Extend to position and velocity
     * 
     * @param Position Position to extend to (in)
     * @param maxSpeed 0 to 1
     */
  

    public CommandBase flaseLimit() {
        return this.runOnce(() -> enableLimit = false);
        }

    public CommandBase trueLimit() {
        return new SequentialCommandGroup(
            this.runOnce(() -> enableLimit = true),
            this.runOnce(() -> setHome()));
    }

    /**
     * Set the home position for the extending motor
     */
    public void setHome() {
        Extend.getEncoder().setPosition(0);
    }

}
