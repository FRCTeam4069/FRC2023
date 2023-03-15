package frc.robot.Auto.Commands.intakeCommands;

import javax.xml.transform.ErrorListener;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class autoWristPose extends CommandBase {
    private static final Intake intake = RobotContainer.intake;
    private double error;
    public autoWristPose() {
        addRequirements(RobotContainer.intake);
    }
    @Override
    public void execute() {
        error = (-3.36 - 0.853 * Math.abs(intake.armAngle) + (0.0103 * Math.pow(Math.abs(intake.armAngle), 2)) - intake.getWristAngle());

        SmartDashboard.putNumber("Auto Pose", (-3.36) + -(0.853 * Math.abs(intake.armAngle)) + (0.0103 * Math.pow(Math.abs(intake.armAngle), 2)));
        if (Math.abs(error) > 10) {
            intake.setWrist( error * (0.01));
        } else {
            intake.setWrist( error * (0.006));
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