package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSelecter extends SubsystemBase {
    private SendableChooser<Integer> autonRoutine;
    private int selectedRoutine;

    public AutoSelecter() {
        ShuffleboardTab auton = Shuffleboard.getTab("Auto");
        autonRoutine = new SendableChooser<Integer>();

        autonRoutine.addOption("Auto Balance (1 cube)", 1);
        
        autonRoutine.addOption("Red Long (2 Game Pieces)", 2);
        autonRoutine.addOption("Red Short (2 Game Pieces)", 3);
        autonRoutine.addOption("Red Long (1 Game Pieces)", 4);
        autonRoutine.addOption("Red Short (1 Game Pieces)", 5);

        autonRoutine.addOption("Blue Long (2 Game Pieces)", 6);
        autonRoutine.addOption("Blue Short (2 Game Pieces)", 7);
        autonRoutine.addOption("Blue Long (1 Game Pieces)", 8);
        autonRoutine.addOption("Blue Short (1 Game Pieces)", 9);

        autonRoutine.addOption("Place Cube", 10);
        autonRoutine.addOption("Middle + Right Cone", 11);
        autonRoutine.addOption("Middle + Left Cone", 12);
        autonRoutine.addOption("Middle + Taxi", 13);


        autonRoutine.setDefaultOption("DO NOTHING", 0);

        auton.add("Autonomous", autonRoutine).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(5,
                5);


    }

    @Override
    public void periodic() {
        selectedRoutine = autonRoutine.getSelected();
    }

    public int getSelected() {
        return selectedRoutine;
    }

}
