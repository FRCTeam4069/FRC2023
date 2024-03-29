package frc.robot.Auto.Commands.intakeAndWristCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intakeSubsystem;

public class OpenIntake extends CommandBase {
    private final intakeSubsystem intake = RobotContainer.intake;
    private final Timer timer;
    public OpenIntake() {
        timer = new Timer();
        addRequirements(RobotContainer.intake);
    }
   
    @Override
    public void execute(){
        intake.ToPose(15);
        timer.start();
    }
    @Override
    public void end(boolean interrupted){
        intake.intakeM2.set(0);
        intake.set(0);

    }
    @Override
    public boolean isFinished(){
        return intake.getPose() > 14;
    }

}