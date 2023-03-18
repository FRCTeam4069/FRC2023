package frc.robot.Constants;

public class States {

    /**
     * For Future use 
     */


    public static enum armState {
        // to see if it is safe to retract arm
        ABOVE_POLES,
        BELOW_POLES,
        START
    }

    public static enum intakeState {
        // To see what certain buttons should do when pressed
        OPEN,
        CLOSE,
        HAS_SOMETHING,
        START
    }

    public static enum extendState {
        // checking if the arm is in, slightly our or almmost all the way out
        RETRACTED,
        EXTENDED_HIGH,
        EXTENDED_MID,
        START
    }

    public static enum robotState {
        // What the robot is currently trying to do
        SCORING_HIGH,
        SCORING_MID,
        HUMAN_PLAYER,
        HOME,
        START
    }

    public static armState currArmState = armState.START;
    public static intakeState currIntakeState = intakeState.START;
    public static extendState currState = extendState.START;
    public static robotState currRobotState = robotState.START;


    
}
