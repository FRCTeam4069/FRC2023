package frc.robot.Auto.routines;

/**
 * Interface for autonomous routines.
 * Created for organizational purposes.
 */
public interface AutoRoutine {

    /**
     * Name of autonomous routine (for auto mode selector)
     * @return Name of routine
     */
    public String name();

    /**
     * Called upon initialization of the autonomous routine
     */
    public void init();

    /**
     * Called approx. every 20ms
     */
    public void loop();

}