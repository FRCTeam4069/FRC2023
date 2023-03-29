package frc.robot.Auto.Commands.intakeCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase {

    private final Intake intake = RobotContainer.intake;
    private final Supplier<Boolean> intakeOpen, intakeClose;
    private final Supplier<Double> wristUp, wristDown;
    public double intakeSpeed, wristPose;
    private Supplier<Integer> POV;

    /**
     * 
     * @param intake
     * @param UP
     * @param DOWN
     * @param OPEN
     * @param CLOSE
     */
    public DefaultIntakeCommand(Supplier<Double> UP, Supplier<Double> DOWN, Supplier<Boolean> OPEN,
            Supplier<Boolean> CLOSE, Supplier<Integer> POV) {
        this.wristUp = UP;
        this.wristDown = DOWN;
        this.intakeOpen = OPEN;
        this.intakeClose = CLOSE;
        this.POV = POV;
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void execute() {

        intake.setWrist((wristUp.get() - wristDown.get()) * armConstants.side);

        if (intakeOpen.get() && intakeClose.get()) {
            intakeSpeed = 0;
        } else if (intakeClose.get()) {
            intakeSpeed = -1;
        } else if (intakeOpen.get()) {
            intakeSpeed = 1;
        } else {
            intakeSpeed = 0;
        }
        intake.setIntake(intakeSpeed);

        if (POV.get() == 0) {
            intake.intakeM2.set(-0.5);
        } else if (POV.get() == 180) {
            intake.intakeM2.set(0.7);
        } else if(intakeClose.get()) {
            intake.intakeM2.set(0.5);
        }else intake.intakeM2.set(0);

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(0);
        intake.setWrist(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
