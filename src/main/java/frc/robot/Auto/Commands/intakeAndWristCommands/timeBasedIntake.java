package frc.robot.Auto.Commands.intakeAndWristCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class timeBasedIntake extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final double power, time, rollerPower;
    private final Timer timer = new Timer();

    public timeBasedIntake(double power, double rollerPower, double time) {
        this.power = power;
        this.time = time;
        this.rollerPower = rollerPower;
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
        intake.set(power);
        intake.intakeM2.set(power);

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        intake.set(0);
        intake.intakeM2.set(0);
    }
}
