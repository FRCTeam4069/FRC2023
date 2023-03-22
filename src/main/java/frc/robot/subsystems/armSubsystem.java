package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;
import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class armSubsystem extends SubsystemBase {
    public CANSparkMax ArticulateR, ArticulateL, Extend;
    public RelativeEncoder leftEncoder, rightEncoder, extendEncoder;
    public boolean enableLimit, _enableLimit, isStuck = false, _isStuck = false;
    public double extendPose = 0, articulatePose = 0, changingArticulateLimits = 130;
    public int articulateLimit = 130;
    public boolean moveToPoseMode = true;
    public Timer stuckTimer = new Timer();

    public armSubsystem() {
        enableLimit = true;
        ArticulateR = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
        ArticulateL = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
        Extend = new CANSparkMax(armConstants.ARM_ID_E, MotorType.kBrushless);

        Extend.setInverted(false);

        // ArticulateR.getEncoder().setPositionConversionFactor(2);
        // Extend.getEncoder().setPositionConversionFactor(0.5/2.33334);

        // ArticulateL.getEncoder().setVelocityConversionFactor(1);
        // ArticulateR.getEncoder().setVelocityConversionFactor(1);

        armConstants.side = getSide();
        setZero();

        ArticulateL.setSoftLimit(SoftLimitDirection.kForward, 130);
        ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -130);
        ArticulateR.setSoftLimit(SoftLimitDirection.kForward, 130);
        ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -130);

        ArticulateL.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kForward, true);
        ArticulateR.enableSoftLimit(SoftLimitDirection.kReverse, true);

        Extend.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Extend.setSoftLimit(SoftLimitDirection.kForward, 24);
        Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
        Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);

        ArticulateR.setInverted(armConstants.rightMotorInvert);
        ArticulateL.setInverted(armConstants.leftMotorInvert);

        Extend.setInverted(armConstants.telescopeMotorInvert);

        ArticulateL.setOpenLoopRampRate(0.5);
        ArticulateR.setOpenLoopRampRate(0.5);

    }

    @Override
    public void periodic() {
        // Update global variables

        armConstants.armPose = articulateDegrees();
        armConstants.extendPose = ExtendedPose();
        armConstants.side = getSide();

        SmartDashboard.putNumber("Side", armConstants.side);

        // Set limits in motor controllers, stops robot from breaking itself
        // only write to motor controllers on change of enable limit to reduce CAN
        // utilization
        if (enableLimit != _enableLimit) {
            Extend.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
            Extend.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
            ArticulateL.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
            ArticulateL.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);
            ArticulateR.setSoftLimit(SoftLimitDirection.kForward, armConstants.softlimits);
            ArticulateR.setSoftLimit(SoftLimitDirection.kReverse, -armConstants.softlimits);

            // Stash last state of enablelimit
            _enableLimit = enableLimit;
        }

        if (enableLimit) {
            extendPose = MathUtil.clamp(extendPose, 0, 24);
            if (-intakeConstants.wristPose * getSide() < 0) {
                articulatePose = MathUtil.clamp(articulatePose, -130, 130);
            } else{
            articulatePose = MathUtil.clamp(articulatePose, -110, 110);//(int) -Math.abs(claculatedLimit()),
}
            //(int) Math.abs(claculatedLimit()));

            SmartDashboard.putNumber("Current Arm Limit", changingArticulateLimits);
            SmartDashboard.putNumber("Current Math",claculatedLimit());
        }
        //armIsNotStuck();
        updateKin();

        moveToPose(articulatePose);
        // extendToPose(extendPose, 1);

        SmartDashboard.putNumber("Arm: Articulate", articulateDegrees());
        SmartDashboard.putNumber("Arm: Target Articulate", articulatePose);
        SmartDashboard.putNumber("Arm: Extend", ExtendedPose());

        SmartDashboard.putNumber("Wrist X", armConstants.x);
        SmartDashboard.putNumber("Wrist Y", armConstants.y);

    }

    private double claculatedLimit() {
        changingArticulateLimits = 180
                - Math.toDegrees(Math.acos((armConstants.L1 / armConstants.L2_OFFSET + armConstants.extendPose)));
        if (-intakeConstants.wristPose * getSide() < 0) {
            changingArticulateLimits -= 10;
        } else {
            changingArticulateLimits -= 20;
        }
        changingArticulateLimits = MathUtil.clamp(Math.abs(changingArticulateLimits), 0, 130);

        return Math.abs(changingArticulateLimits);
    }

    private boolean updateKin() {
        armConstants.c2 = Math.cos(Math.toRadians(armConstants.armPose + 90));
        armConstants.s2 = Math.sin(Math.toRadians(armConstants.armPose + 90));
        armConstants.c3 = Math.cos(Math.toRadians(intakeConstants.wristPose));
        armConstants.s3 = Math.sin(Math.toRadians(intakeConstants.wristPose));

        // Update length of L2
        armConstants.L2 = armConstants.L2_OFFSET + armConstants.extendPose;

        armConstants.x = armConstants.L3
                * (armConstants.c1 * armConstants.c2 * armConstants.c3
                        - armConstants.c1 * armConstants.s2 * armConstants.s3)
                + armConstants.c1 * armConstants.c2 * armConstants.L2;

        armConstants.y = armConstants.L3 * (armConstants.s2 * armConstants.c3 + armConstants.c2 * armConstants.s3)
                + armConstants.s2 * armConstants.L2 + armConstants.L1;

        return false;
    }

    private boolean moveToPose(double pose) {

        double speed = MathUtil.clamp(
                ((pose - articulateDegrees()) * armConstants.proportionalGain
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

        if ((speed * getSide() > 0) && (armConstants.y > armConstants.YLIMIT)) { // moving arm up but we're at the limit
            // return;
        }

        ArticulateL.set(speed);
        ArticulateR.set(speed);

        if ((-intakeConstants.wristPose * getSide() < 0)) {
            articulateLimit = 130;
        } else {
            articulateLimit = 110;
        }
    }

    public void manualExtend(double speed) {
        if ((speed > 0) && (armConstants.x > armConstants.XLIMIT)) { // moving arm out but we're at the limit
            // return;
        }

        if (ExtendedPose() < 2 && speed < -0.25) {
            Extend.set(speed * 0.25);
        } else if (ExtendedPose() > 22 && speed > 0.25) {
            Extend.set(speed * 0.25);
        } else {
            Extend.set(speed);
        }

    }

    public void setArmPose(double pose) {
        this.articulatePose = pose;
    }

    public void setExtendPose(double pose) {
        this.extendPose = pose;
    }

    public boolean isAtPoseAT(double Pose, double threshold) {
        return (Math.abs(Pose - articulateDegrees()) < threshold);

    }

    public boolean isAtPoseE(double Pose, double threshold) {
        return (Math.abs(Pose - ExtendedPose()) < threshold);
    }

    /**
     * Arm Pose in Degrees
     * 
     * @return
     */
    public double articulateDegrees() {
        return ArticulateR.getEncoder().getPosition();
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
        return Math.copySign(1, articulateDegrees());
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

    public void updtaeArmStates() {

    }

    /**
     * {@summary Stops the arm form trying to break itself if it is stuck}
     */
    public void armIsNotStuck() {
        // check if the arm is at its target position
        // or atleast within a threshold fo the target position
        // if arm if not then start a timer
        // if timer gets past certain point then
        // set the target position to current position

        if (!isAtPoseAT(articulatePose, 5)) {
            stuckTimer.start();
            if (stuckTimer.hasElapsed(3) && !isAtPoseAT(articulatePose, 5)) {
                isStuck = true;
                articulatePose = articulateDegrees();
            }
        } else if (!isAtPoseAT(articulatePose, 130)) {
            // Stops the arm target position to current position difference form ever being
            // more then 130 degrees
            // Theoretically stopping it form ever bashing into the ground...
            isStuck = true;
            articulatePose = articulateDegrees();
        } else {
            stuckTimer.reset();
            stuckTimer.stop();
        }

        if (isStuck != _isStuck) {
            System.out.println("Arm Got Stuck... @Position: " + articulateDegrees()
                    + "\n Arm Target Position has been reset to :" + articulatePose);
            isStuck = false;
            _isStuck = isStuck;
        }

    }

}
