package frc.robot.Constants;

import java.util.HashMap;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.Commands.armCommands.ArmRoutines.HomePose;
import frc.robot.Auto.Commands.intakeCommands.timeBasedIntake;
import frc.robot.Auto.routines.placeCubeL3;

public class AutoValues {
    public static class autonArmValues{

    }
    public static class autonSwerveValues{
        
    } 
    public static class autonZIntakeValues{
        
    } 
    public static class autonCameraValues{
        
    }
     
    public static HashMap<String, Command> eventMap = new HashMap<>();
    public AutoValues(){
    eventMap.put("CloseIntake", new timeBasedIntake(0.5, 1));
    eventMap.put("CUBE_L3", new placeCubeL3());
    eventMap.put("ARM_DOWN", new HomePose());
}
}
