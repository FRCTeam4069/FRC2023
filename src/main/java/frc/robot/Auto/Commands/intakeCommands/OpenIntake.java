package frc.robot.Auto.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class OpenIntake extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final double targetPose;

    public OpenIntake(double pose) {
        this.targetPose = pose;
        intake.setIntakePose(16.8 - 4.3);
        addRequirements(RobotContainer.intake);
    }
    @Override
    public void execute(){
        intake.intakeToPose(16);


    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return intake.intakeIsAtPose(16,0.1);
    }

}