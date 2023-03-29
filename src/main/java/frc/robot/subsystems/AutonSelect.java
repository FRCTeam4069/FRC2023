package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonSelect extends SubsystemBase {
    private SendableChooser<Integer> autonRoutine;
    private int selectedRoutine;

    public AutonSelect() {
        ShuffleboardTab auton = Shuffleboard.getTab("Auto");
        autonRoutine = new SendableChooser<Integer>();

        autonRoutine.addOption("autoBalance", 0);
        autonRoutine.addOption("RED_LONG", 1);
        autonRoutine.addOption("RED_SHORT", 2);
        autonRoutine.addOption("BLUE_LONG", 3);
        autonRoutine.addOption("BLUE_SHORT", 4);
        autonRoutine.addOption("CUBE_L3", 5);
        autonRoutine.addOption("TEST", 7);
        autonRoutine.setDefaultOption("DO NOTHING", 6);

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
