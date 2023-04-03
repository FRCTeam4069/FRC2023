package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class intakeSubsystem extends SubsystemBase {
    public CANSparkMax intake, intakeM1, intakeM2;
    private RelativeEncoder intakeEncoder;
    public DigitalInput NeoSideLimit, Limit;
    public edu.wpi.first.wpilibj.AnalogInput PhotoElectric;
    public GenericEntry hasCone;
    public boolean enableLimit = true, _enableLimit = false, coneInRange, _coneInRange = false;
    public ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    public Timer itsRUMBLEtime = new Timer();

    public intakeSubsystem() {
        hasCone = tab.add("Intake has Somethig", coneInRange).getEntry();

        intake = new CANSparkMax(intakeConstants.INTAKE_ID, MotorType.kBrushless);
        intakeM1 = new CANSparkMax(21, MotorType.kBrushless);
        intakeM2 = new CANSparkMax(22, MotorType.kBrushless);
        NeoSideLimit = new DigitalInput(armConstants.NEO_LIMIT);
        Limit = new DigitalInput(armConstants.LIMIT);
        PhotoElectric = new edu.wpi.first.wpilibj.AnalogInput(armConstants.PHOTOELECTRIC);

        intakeM1.follow(intakeM2);

        intake.setSmartCurrentLimit(40);
        intakeEncoder = intake.getEncoder();

        intakeEncoder.setPosition(0);// 16.8 - 4.3);

        // setIntakePose(0);
        intake.setSoftLimit(SoftLimitDirection.kReverse, 0);
        intake.setSoftLimit(SoftLimitDirection.kForward, 15);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        isConeInRange();

        atLimit();
        // setIntakeMotors();
        if (enableLimit != _enableLimit) {
            intake.enableSoftLimit(SoftLimitDirection.kForward, enableLimit);
            intake.enableSoftLimit(SoftLimitDirection.kReverse, enableLimit);
            _enableLimit = enableLimit;
        }
        hasCone.setBoolean(coneInRange);
    }

    public boolean isConeInRange() {
        if (PhotoElectric.getVoltage() < 0.26) {
            coneInRange = true;
            return true;
        } else {
            coneInRange = false;
            return false;
        }
    }

    public void atLimit() {
        if (!NeoSideLimit.get() || !Limit.get()) {
            intakeEncoder.setPosition(15);
        } else {

        }

    }

    public void ToPose(double position) {
        set((position - getPose()) * intakeConstants.intakekP);
    }

    public void set(double speed) {
        intake.set(speed);
    }

    public double getPose() {
        return intakeEncoder.getPosition();
    }

    public void setPose(double pose) {
        intakeEncoder.setPosition(pose);
    }

    public CommandBase zero(double pose) {
        return this.runOnce(() -> intakeEncoder.setPosition(pose));
    }

    public CommandBase enableLimit(boolean onORoff) {
        return this.runOnce(() -> enableLimit = onORoff);
    }

}
