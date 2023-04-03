package frc.robot.Auto.Commands.intakeAndWristCommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.wristSubsystem;

public class autoWristParallel extends CommandBase {
    private static final wristSubsystem intake = RobotContainer.wrist;
    private double error;
    public autoWristParallel() {
        addRequirements(RobotContainer.intake);
    }
    @Override
    public void execute() {
        //error = (-3.36 - 0.853 * Math.abs(intake.armAngle) + (0.0103 * Math.pow(Math.abs(intake.armAngle), 2)) - intake.getWristAngle());
        error = (-intake.parallelAngle) - intake.getWristAngle();
        if (Math.abs(error) > 10) {
            intake.setWrist( error * (0.01));
        } else {
            intake.setWrist( error * (0.02));
        }


    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}