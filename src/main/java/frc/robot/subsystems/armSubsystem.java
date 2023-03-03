package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.armAndIntakeConstants.armConstants;;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    public RelativeEncoder leftEncoder, rightEncoder, extendEncoder;
    public boolean enableLimit;
    public double extendPose = 0, articulatePose = 0;
    private ShuffleboardTab tab;

    public armSubsystem() {
        enableLimit = true;
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        Extend.setInverted(false);

        ArticulateL.getEncoder().setPositionConversionFactor(2);
        ArticulateR.getEncoder().setPositionConversionFactor(2);
        Extend.getEncoder().setPositionConversionFactor((1 / 392) * 0.5);

        setZero();
        ArticulateL.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
        ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);

        ArticulateL.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kReverse, true);

        Extend.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Extend.setSoftLimit(SoftLimitDirection.kForward, 130);
        Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);

        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);

        Extend.setInverted(armConstants.telescopeMotorInvert);

        tab =  Shuffleboard.getTab("Debug");

    }

    @Override
    public void periodic() {
        Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);

        MathUtil.clamp(extendPose, 0, 140);
        MathUtil.clamp(articulatePose, -130, 130);

        moveToPose(articulatePose);
        extendToPose(extendPose, 0.5);
        if(IO.PrintIntakeData == true && IO.DISABLE_ALL_SMARTDASH_DATA == false){
            tab.add("Right Pose", rightMotorPosition());
            tab.add("Left Pose", leftMotorPosition());
            tab.add("Number of Ticks Extend", Extend.getEncoder().getCountsPerRevolution());
            tab.add("Number of Ticks Ar", ArticulateL.getEncoder().getCountsPerRevolution());
            tab.add("Number of Ticks Al", ArticulateR.getEncoder().getCountsPerRevolution());
            tab.add("Lead Screw Rotations: ", ExtendedPose());
        }
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
        double speed = MathUtil.clamp((Position - ExtendedPose()) * 0.2, -maxSpeed, maxSpeed);
        manualExtend(speed);

    }

    public void manualArticulate(double speed) {
        ArticulateL.set(speed);
        ArticulateR.set(speed);
    }

    public void manualExtend(double speed) {
        Extend.set(-speed);
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
    }

    public double rightMotorPosition() {
        return ArticulateR.getEncoder().getPosition();
    }

    public double leftMotorPosition() {
        return ArticulateR.getEncoder().getPosition();
    }

    public double AvgPose() {
        return ((ArticulateR.getEncoder().getPosition() + ArticulateL.getEncoder().getPosition()) / 2);
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
        return new SequentialCommandGroup(
                this.runOnce(() -> enableLimit = false),
                this.runOnce(() -> setHome()));
    }

    public CommandBase trueLimit() {
        return this.runOnce(() -> enableLimit = true);
    }

    /**
     * Set the home position for the extending motor
     */
    public void setHome() {
        Extend.getEncoder().setPosition(0);
    }

}
