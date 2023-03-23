package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IO;

public class Debugger extends SubsystemBase{
    public ShuffleboardTab debugTab;
    public GenericEntry arm, intake, swerve, gyro, A, S, I, G, disbaleAll, D;

    public Debugger(){
        debugTab = Shuffleboard.getTab("Debug");   
        disbaleAll = debugTab.add("Disable all?", IO.PrintArmData).withPosition(0, 0).withSize(5, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        arm = debugTab.add("Print Arm Numbers", IO.PrintArmData).withPosition(0, 1).withSize(4, 1).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        intake = debugTab.add("Print Intake Numbers", IO.PrintIntakeData).withPosition(0, 3).withSize(4, 1).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        swerve = debugTab.add("Print Swerve Numbers", IO.PrintSwerveData).withPosition(0, 2).withSize(4, 1).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        gyro = debugTab.add("Print Gyro Numbers", IO.PrintGryoData).withPosition(0, 4).withSize(4, 1).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        
        A = debugTab.add("Arm? ", IO.PrintArmData).withPosition(4, 1).getEntry();
        S= debugTab.add("Swerve? ", IO.PrintSwerveData).withPosition(4, 2).getEntry();
        I = debugTab.add("Intake? ", IO.PrintIntakeData).withPosition(4,3).getEntry();
        G = debugTab.add("Gyro? ", IO.PrintGryoData).withPosition(4, 4).getEntry();

    } 
    
    @Override
    public void periodic(){
        
        IO.PrintGryoData = gyro.get().getBoolean() && !disbaleAll.get().getBoolean();
        IO.PrintSwerveData = swerve.get().getBoolean() && !disbaleAll.get().getBoolean();
        IO.PrintArmData = arm.get().getBoolean() && !disbaleAll.get().getBoolean();
        IO.PrintIntakeData = intake.get().getBoolean() && !disbaleAll.get().getBoolean();
        A.setBoolean(IO.PrintArmData);
        I.setBoolean(IO.PrintIntakeData);
        S.setBoolean(IO.PrintSwerveData);
        G.setBoolean(IO.PrintGryoData);
        

    }
    
}
