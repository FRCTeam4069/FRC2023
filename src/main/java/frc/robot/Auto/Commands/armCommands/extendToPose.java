package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class extendToPose extends CommandBase {
    private final double targetPose, threshold;
    private static final armSubsystem arm = RobotContainer.arm;

    public extendToPose(double position, double threshold) {
        this.targetPose = position;
        this.threshold = threshold;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        // arm.setExtendPose(MathUtil.clamp(targetPose, 0, 130));
        arm.extendToPose(targetPose, 1);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (threshold != -1) {
            return arm.isAtPoseE(targetPose, threshold);
        } else
            return false;

    }

}
