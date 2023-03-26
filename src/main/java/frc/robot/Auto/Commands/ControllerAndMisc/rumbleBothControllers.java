package frc.robot.Auto.Commands.ControllerAndMisc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class rumbleBothControllers extends CommandBase{
    private static final XboxController C1 = RobotContainer.Controller1;
    private static final XboxController C2 = RobotContainer.Controller2;
    private final double power, time;
    private final Timer timer = new Timer();

    public rumbleBothControllers(double power, double timer){
        this.power = power;
        this.time = timer;
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.stop();

    }
    @Override
    public void execute(){
        timer.start();
        C1.setRumble(RumbleType.kBothRumble, power);
        C2.setRumble(RumbleType.kBothRumble, power);

    }
    @Override
    public void end(boolean interrupted){
        C1.setRumble(RumbleType.kBothRumble, 0);
        C2.setRumble(RumbleType.kBothRumble, 0);

    }
    @Override
    public boolean isFinished(){
        return timer.hasElapsed(time);
    }
    
}
