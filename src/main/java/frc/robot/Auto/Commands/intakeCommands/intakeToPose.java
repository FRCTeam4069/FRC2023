package frc.robot.Auto.Commands.intakeCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class intakeToPose extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final Timer timer;
    private final double RollerSpeeds;
    private final double target, threshold, timeout;
    public intakeToPose(double target,double threshold,double timeout, double RollerSpeeds) {
        this.target = target;
        this.threshold = threshold;
        this.RollerSpeeds = RollerSpeeds;
        this.timeout = timeout;
        timer = new Timer();
        addRequirements(RobotContainer.intake);
    }
   
    @Override
    public void initialize(){
        timer.reset();
        timer.stop();
    }
    

    @Override
    public void execute(){
        intake.intakeToPose(target);
        intake.intakeM2.set(RollerSpeeds);
        timer.start();
    }

    @Override
    public void end(boolean interrupted){
        intake.intakeM2.set(0);
        intake.setIntake(0);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(intake.getIntakePose() - target) < threshold || timer.hasElapsed(timeout);
    }

}