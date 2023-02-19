package frc.robot.Auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Auto.Scheduler;
import frc.robot.Auto.Commands.drivebaseCommands.followTrajectoryCommand;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public class testRoutine implements AutoRoutine {
    private Scheduler scheduler;
    private RobotContainer Robot;

    
    public testRoutine(Robot robot){
        Robot = frc.robot.Robot.m_robotContainer;
        scheduler = new Scheduler(robot);
    }

    /**
     * Name of autonomous routine (for auto mode selector)
     * @return Name of routine
     */
    public String name(){
        return "Test";

    }

    /**
     * Called upon initialization of the autonomous routine
     */
    public void init(){
        scheduler.addCommand(new followTrajectoryCommand(Robot.swerveSubsystem, PathPlanner.loadPath("Go to Cone", new PathConstraints(0.5, 1)) , true));
        
    }

    /**
     * Called approx. every 20ms
     */
    public void loop(){
        scheduler.run();

    }

}