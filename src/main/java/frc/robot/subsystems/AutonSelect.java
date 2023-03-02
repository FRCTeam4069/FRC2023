package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armAndIntakeConstants.intakeConstants;

public class AutonSelect extends SubsystemBase {
    private SendableChooser<Integer> autonRoutine;
    private int selectedRoutine;

    public AutonSelect(){
        ShuffleboardTab auton = Shuffleboard.getTab("Auto");
        autonRoutine = new SendableChooser<Integer>();

        autonRoutine.addOption("autoBalance", 0);
        autonRoutine.addOption("1", 1);
        autonRoutine.addOption("2", 2);
        autonRoutine.addOption("3", 3);
        autonRoutine.addOption("Leave Community", 4);
        autonRoutine.setDefaultOption("Cone + Leave Community", 5);

        auton.add("Autonomous", autonRoutine).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
    }

    @Override
    public void periodic(){
        selectedRoutine = autonRoutine.getSelected();

        
    }

    public int getSelected(){
        return selectedRoutine;
    }

    
}
