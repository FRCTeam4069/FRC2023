package frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IO{
    public static boolean PrintIntakeData = true;
    public static boolean PrintSwerveData = true;
    public static boolean PrintArmData = true;
    public static boolean PrintGryoData = true;
    public static boolean DISABLE_ALL_SMARTDASH_DATA = true;
    
    public static boolean enableSlewrateLimiter = true;
    public static double driveDeadband;

    public static final double Contoller1portNum = 0;
    public static final double maxTurnSpeed = 5; // Meters/sec
    public static final double maxSpeed = 5; // Meters/sec

    public static final double xdeadZone = 0.07;
    public static final double ydeadZone = 0.07;
    public static final double tdeadZone = 0.07;

    public static final double xSpeedSlewRate = 3;
    public static final double ySpeedSlewRate = 3;
    public static final double TurnSpeedSlewRate = 3;

    

}