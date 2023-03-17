package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class armRelativeArmToPose extends CommandBase {
    private final double change, threshold;
    private double targetPose;

    private static final armSubsystem arm = RobotContainer.arm;

    public armRelativeArmToPose(double change, double threshold) {
        this.change = change;
        this.threshold = threshold;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        targetPose = arm.articulatePose + (change *arm.getSide()); 
    }
         
    @Override
    public void execute() {
         
        arm.setArmPose(arm.articulatePose + (change *arm.getSide()));

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return arm.isAtPoseAT(targetPose, threshold);

    }

}

