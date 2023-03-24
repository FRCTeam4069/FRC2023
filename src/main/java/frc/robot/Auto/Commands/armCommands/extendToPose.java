package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("Extend Error", targetPose - arm.ExtendedPose());
    }

    @Override
    public void end(boolean interrupted) {
        arm.Extend.set(0);

    }

    @Override
    public boolean isFinished() {
        return arm.isAtPoseE(targetPose, threshold);
    }

}
