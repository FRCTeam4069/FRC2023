package frc.robot.Auto.Commands.intakeAndWristCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wristSubsystem;

public class scalableWristToPosition extends CommandBase {
    private static final wristSubsystem wrist = RobotContainer.wrist;
    private final double threshold, targetPose, thresholdTime, timeOut, speedClamp;
    private final Timer timeOutTimer = new Timer(), thresholdTimer = new Timer();

    public scalableWristToPosition(double targetPose, double threshold, double thresholdTime, double timeOut, double speedClamp) {
        this.timeOut = timeOut;
        this.threshold = threshold;
        this.thresholdTime = thresholdTime;
        this.speedClamp = speedClamp;
        this.targetPose = MathUtil.clamp(targetPose, -100, 100);
        addRequirements(RobotContainer.wrist);
    }

    @Override
    public void initialize() {
        timeOutTimer.restart();
        wrist.setHoldPose(targetPose * wrist.side);
    }

    @Override
    public void execute() {
        
        if (Math.abs(targetPose - wrist.getWristAngle()) > 10) {
            wrist.setWrist(MathUtil.clamp((targetPose * wrist.side - wrist.getWristAngle()) * (0.019), -speedClamp, speedClamp));
        } else {
            wrist.setWrist(MathUtil.clamp((targetPose * wrist.side - wrist.getWristAngle()) * (0.01), -speedClamp, speedClamp));
    
        }

        if (Math.abs(targetPose - wrist.getWristAngle()) < threshold) {
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