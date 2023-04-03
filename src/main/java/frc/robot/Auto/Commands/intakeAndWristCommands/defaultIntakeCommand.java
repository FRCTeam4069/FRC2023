package frc.robot.Auto.Commands.intakeAndWristCommands;


import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.intakeSubsystem;

public class defaultIntakeCommand extends CommandBase {

    private final intakeSubsystem intake = RobotContainer.intake;
    private final Supplier<Boolean> intakeOpen, intakeClose;
    public double intakeSpeed, wristPose;
    private Supplier<Integer> POV;

    public defaultIntakeCommand(Supplier<Boolean> OPEN,Supplier<Boolean> CLOSE, Supplier<Integer> POV) {
        this.intakeOpen = OPEN;
        this.intakeClose = CLOSE;
        this.POV = POV;
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void execute() {

        if (intakeOpen.get() && intakeClose.get()) {
            intakeSpeed = 0;
        } else if (intakeClose.get()) {
            intakeSpeed = -1;
        } else if (intakeOpen.get()) {
            intakeSpeed = 1;
        } else {
            intakeSpeed = 0;
        }

        intake.set(intakeSpeed);

        if (POV.get() == 0) {
            //when dpad up, rollers outtake
            intake.intakeM2.set(-0.5);
        } else if (POV.get() == 180) {
            //when dpad down, rollers intake
            intake.intakeM2.set(0.7);
        } else if (intakeClose.get()) {
            //when intake close, rollers intake
            intake.intakeM2.set(0.5);
        } else
            intake.intakeM2.set(0);


    }


    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}
