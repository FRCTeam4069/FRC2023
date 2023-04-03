package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class wristSubsystem extends SubsystemBase {
    public CANSparkMax  wrist;
    private RelativeEncoder wristEncoder;
    public double side, wristTarget, low, gravGain, armAngle, parallelAngle;
    public boolean enableLimit = true, _enableLimit = false;

    public wristSubsystem() {
        wrist = new CANSparkMax(intakeConstants.WRIST_ID, MotorType.kBrushless);

        wrist.setSmartCurrentLimit(40);
        wrist.setInverted(false);
        wrist.setOpenLoopRampRate(0);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, -34);
        wrist.setSoftLimit(SoftLimitDirection.kForward, 34 );
        wrist.setIdleMode(IdleMode.kBrake);
        wristEncoder = wrist.getEncoder();
        wristEncoder.setPositionConversionFactor(1);
        wristEncoder.setVelocityConversionFactor(1);

        /*
         * wrist is 70:1
         * meaning 2940 encoder ticks : 1 wrist rotation
         * 1/2940
         */

        SmartDashboard.putNumber("wrist", wristEncoder.getPositionConversionFactor());

        setWristPose(40);

        // setIntakePose(0);

    }

    @Override
    public void periodic() {
        intakeConstants.wristPose = getWristAngle();
        side = armConstants.side;
        armAngle = armConstants.armPose;
        SmartDashboard.putNumber("Wrist angle", getWristAngle());


        if (enableLimit != _enableLimit) {
            wrist.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
            wrist.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
            _enableLimit = enableLimit;
        }
    
        parallelAngle = low - (side * 90);
        low = (180 - Math.abs(armConstants.armPose)) * armConstants.side;

    }

    public double getWristAngle() {
        return (getWristPose() * 2 / 80 * 120);
    }

    public void setWrist(double speed) {
        wrist.set(speed);
    }

    public double getWristPose() {
        return wristEncoder.getPosition();
    }

    public void setWristPose(double pose) {
        wristEncoder.setPosition(pose);
    }

    public CommandBase zeroWrist() {
        return this.runOnce(() -> setWristPose(0));
    }
    public CommandBase enableLimit(boolean onORoff) {
        return this.runOnce(() -> enableLimit = onORoff);
    }

}
