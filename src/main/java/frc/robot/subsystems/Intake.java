package frc.robot.subsystems;

import org.opencv.photo.Photo;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.States;
import frc.robot.Constants.States.intakeState;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax intake, wrist;
    private RelativeEncoder intakeEncoder, wristEncoder;
    public double side, wristTarget, low, gravGain, armAngle, parallelAngle;
    public DigitalInput NeoSideLimit, Limit;
    public edu.wpi.first.wpilibj.AnalogInput PhotoElectric;
    public GenericEntry hasCone;
    public boolean enableLimit = true, _enableLimit = false, coneInRange;
    public ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    public Intake() {
        hasCone = tab.add("Intake has Somethig", coneInRange).getEntry();

        intake = new CANSparkMax(intakeConstants.INTAKE_ID, MotorType.kBrushless);
        wrist = new CANSparkMax(intakeConstants.WRIST_ID, MotorType.kBrushless);
        NeoSideLimit = new DigitalInput(armConstants.NEO_LIMIT);
        Limit = new DigitalInput(armConstants.LIMIT);
        PhotoElectric = new edu.wpi.first.wpilibj.AnalogInput(armConstants.PHOTOELECTRIC);

        intake.setSmartCurrentLimit(40);
        wrist.setSmartCurrentLimit(40);
        wrist.setInverted(false);
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
        intakeEncoder.setPosition(0);// 16.8 - 4.3);

        // setIntakePose(0);
        intake.setSoftLimit(SoftLimitDirection.kReverse, 0);
        intake.setSoftLimit(SoftLimitDirection.kForward, 15);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, -40);
        wrist.setSoftLimit(SoftLimitDirection.kForward, 39);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        isConeInRange();
        atLimit();
        intakeConstants.wristPose = getWristAngle();
        side = armConstants.side;
        armAngle = armConstants.armPose;
        if (enableLimit != _enableLimit) {
            intake.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
            intake.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
            wrist.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
            wrist.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
            _enableLimit = enableLimit;
        }

        SmartDashboard.putNumber("Intake Pose", getIntakePose());
        SmartDashboard.putNumber("Wrist Subsystem received Arm side", side);
        SmartDashboard.putNumber("wrist angle", getWristAngle());
        SmartDashboard.putNumber("wrist Target (Subsystem)", wristTarget);
        SmartDashboard.putNumber("PhotoElectric V", PhotoElectric.getVoltage());
        SmartDashboard.putBoolean("Limit", !Limit.get());
        SmartDashboard.putBoolean("NeoLimit", !NeoSideLimit.get());
        low = (180 - Math.abs(armConstants.armPose)) * armConstants.side;
        SmartDashboard.putNumber("Low", low);
        SmartDashboard.putNumber("gravGain t1", ((getWristAngle() - low) * 0.01));
        SmartDashboard.putNumber("gravGain t2 (sin)", Math.sin(Math.toRadians(getWristAngle() - low)) * 0.1);// * (0 -
                                                                                                             // getWristAngle()));
        SmartDashboard.putNumber("gravGain t3 (cos)", Math.cos(Math.toRadians(getWristAngle() - low)));
        SmartDashboard.putNumber("parallel angle", low - (side * 90));
        parallelAngle = low - (side * 90);

        hasCone.setBoolean(coneInRange);

    }

    // +iv is up in negitive Side (arm)
    // -iv is up in positive Side (arm)

    public void isConeInRange() {
        if (PhotoElectric.getVoltage() > 0.26) {
            coneInRange = true;
        } else
            coneInRange = false;
    }

    public void atLimit() {
        if (!NeoSideLimit.get() || !Limit.get()) {
            intakeEncoder.setPosition(15);
        }
        else{

        }

    }

    public double getWristAngle() {
        return (getWristPose() * 2 / 80 * 120);
    }

    public void intakeToPose(double position) {
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

    public CommandBase diableLimit() {
        return this.runOnce(() -> enableLimit = false);
    }

    public CommandBase enableLimit() {
        return this.runOnce(() -> enableLimit = true);
    }

    public void updateState() {
        double openPose = 16;
        double closePose = 10;
        if (getIntakePose() < openPose) {
            States.currIntakeState = intakeState.OPEN;
        }
    }

}
