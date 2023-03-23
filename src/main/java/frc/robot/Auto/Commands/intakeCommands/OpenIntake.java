package frc.robot.Auto.Commands.intakeCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class OpenIntake extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final Timer timer;
    public OpenIntake() {
        timer = new Timer();
        addRequirements(RobotContainer.intake);
    }
   
    @Override
    public void execute(){
        intake.intakeToPose(15);
        timer.start();
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return intake.getIntakePose() > 14;
    }

}