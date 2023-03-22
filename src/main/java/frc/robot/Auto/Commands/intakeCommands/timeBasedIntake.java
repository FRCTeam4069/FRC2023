package frc.robot.Auto.Commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Auto.Commands.Command;
import frc.robot.subsystems.Intake;

public class timeBasedIntake extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final double power, time;
    private final Timer timer = new Timer();

    public timeBasedIntake(double power, double time) {
        this.power = power;
        this.time = time;
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize() {
    timer.reset();
    timer.stop();
    }

    @Override
    public void execute() {
        timer.start();
        intake.setIntake(power);

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
