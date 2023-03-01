package frc.robot.Auto;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Scheduler {
    private List<CommandBase> commands = new ArrayList<CommandBase>();
    protected Robot robot;
    public boolean started = false;


    public Scheduler(Robot robot){
        this.robot = robot;
    }

    public void addCommand(CommandBase command) {
        commands.add(command);
    }
    public void setCommands(int index, CommandBase command){
        commands.add(index, command);
    }

    public int getListSize(){
        return commands.size();
    }

    public void run(){
        if(getListSize() != 0){
            CommandBase nextCommand = commands.get(0);

            if (!started){
                nextCommand.execute();
                started = true;
            }

            nextCommand.execute();
            if (nextCommand.isFinished()){
                nextCommand.end(false);
                commands.remove(0);
                if(commands.size() != 0) {
                    commands.get(0).execute();
                }
            }
        }
        else{
            disableSubsystems();
        }
    }
    public void disableSubsystems() {

    }
}

