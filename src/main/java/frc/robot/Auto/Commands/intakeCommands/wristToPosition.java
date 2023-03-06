package frc.robot.Auto.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class wristToPosition extends CommandBase {
    private static final Intake intake = RobotContainer.intake;
    private final double targetPose;

    public wristToPosition(double pose) {
        this.targetPose = pose;
        addRequirements(RobotContainer.intake);
    }
    @Override
    public void execute(){
        intake.wristToPose(targetPose);
    }
    @Override
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished(){
        return intake.wristIsAtPose(targetPose, 0.1);

    }

}