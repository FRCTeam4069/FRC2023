package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class fieldRelativeArmToPose extends CommandBase {
    private final double targetPose, threshold;
    private static final armSubsystem arm = RobotContainer.arm;
    

    public fieldRelativeArmToPose(double position, double threshold) {
        this.targetPose = position;
        this.threshold = threshold;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        arm.setArmPose(targetPose * RobotContainer.swerveSubsystem.getSide());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return arm.isAtPoseAT(targetPose * RobotContainer.swerveSubsystem.getSide(), threshold);
    }

}

