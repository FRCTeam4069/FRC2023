package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class armToPose extends CommandBase {
    private final double targetPose, threshold;
    private final boolean armOriented;

    private static final armSubsystem arm = RobotContainer.arm;

    public armToPose(double position, Boolean armOriented, double threshold) {
        this.targetPose = position;
        this.threshold = threshold;
        this.armOriented = armOriented;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        if(armOriented) arm.setArmPose(targetPose*arm.getSide());
        else arm.setArmPose(targetPose);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(armOriented){
        return arm.isAtPoseAT(targetPose*arm.getSide(), threshold);
        } else return arm.isAtPoseAT(targetPose, threshold);

    }

}

