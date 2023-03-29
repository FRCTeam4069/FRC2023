package frc.robot.Auto.Commands.intakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class wristToPosition extends CommandBase {
    private static final Intake intake = RobotContainer.intake;
    private final double threshold, targetPose, thresholdTime, timeOut;
    private final Timer timeOutTimer = new Timer(), thresholdTimer = new Timer();

    public wristToPosition(double targetPose, double threshold, double thresholdTime, double timeOut) {
        this.timeOut = timeOut;
        this.threshold = threshold;
        this.thresholdTime = thresholdTime;
        this.targetPose = MathUtil.clamp(targetPose, -100, 100);
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize(){
        timeOutTimer.restart();
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("Wrist (Side)", intake.side);
        SmartDashboard.putNumber("Wrist (Target)", targetPose);
        SmartDashboard.putNumber("Wrist (power)", (targetPose * intake.side - intake.getWristAngle()) * (0.01));
        SmartDashboard.putNumber("Threshold Time", thresholdTimer.get());
        SmartDashboard.putNumber("Timout Time", timeOutTimer.get());
        SmartDashboard.putBoolean("Is finished", thresholdTimer.hasElapsed(thresholdTime) || timeOutTimer.hasElapsed(timeOut));

        if (Math.abs(targetPose - intake.getWristAngle()) > 10) {
            intake.setWrist((targetPose * intake.side - intake.getWristAngle()) * (0.019));
        } else {
            intake.setWrist((targetPose * intake.side - intake.getWristAngle()) * (0.01));
        }

        if (Math.abs(targetPose - intake.getWristAngle()) < threshold) {
            thresholdTimer.start();
        } else {
            thresholdTimer.reset();
            thresholdTimer.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return thresholdTimer.hasElapsed(thresholdTime) || timeOutTimer.hasElapsed(timeOut);
    }

}