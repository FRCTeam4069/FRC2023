package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonSelect extends SubsystemBase {
    private SendableChooser<Integer> autonRoutine;
    private int selectedRoutine;

    public AutonSelect(){
        ShuffleboardTab auton = Shuffleboard.getTab("Auto");
        autonRoutine = new SendableChooser<Integer>();

        autonRoutine.addOption("autoBalance", 0);
        autonRoutine.setDefaultOption("Shortside", 1);
        autonRoutine.addOption("Longside", 2);
        auton.add("Autonomous", autonRoutine).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(5,5);
    }

    @Override
    public void periodic(){
        selectedRoutine = autonRoutine.getSelected();        
    }

    public int getSelected(){
        return selectedRoutine;
    }

    
}
