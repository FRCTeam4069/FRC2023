package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class moveToPose extends CommandBase {
    private final double targetPose;
    private final boolean armOriented;

    private static final armSubsystem arm = RobotContainer.arm;

    public moveToPose(double position, Boolean armOriented) {
        this.targetPose = position;
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
        // if(threshold != -1){
        return arm.isAtPoseAT(targetPose, 1);
        // } else return false;
        //return true;

    }

}

