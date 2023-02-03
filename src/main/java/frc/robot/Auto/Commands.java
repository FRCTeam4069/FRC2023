package frc.robot.Auto;

import frc.robot.Robot;

public abstract class Commands {
    protected Robot robot;
    
    public void setSubsystems(Robot robo){
        robot = robo;
    }

    public abstract void start();
    public abstract void execute();
    public abstract boolean isFinished();
    public abstract void shutDownCommand();
}
