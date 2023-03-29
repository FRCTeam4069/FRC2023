package frc.robot.Auto.Commands.ControllerAndMisc;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrintCommand extends CommandBase{
    private final String printThis;

    public PrintCommand(String PrintThis){
        this.printThis = PrintThis;
    }

    @Override
    public void execute() {
        System.out.println(printThis);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
