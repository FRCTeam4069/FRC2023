package frc.robot.Auto.Commands.intakeAndWristCommands;

import java.util.function.Supplier;

import javax.swing.GroupLayout;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.wristSubsystem;

public class defaultWristCommand extends CommandBase {

    private final wristSubsystem wrist = RobotContainer.wrist;
    private final Supplier<Double> wristUp, wristDown;
    public double power, targetPose;
    private boolean hold = true, _hold = false;

    /**
     * 
     * @param UP
     * @param DOWN
     */
    public defaultWristCommand(Supplier<Double> UP, Supplier<Double> DOWN) {
        this.wristUp = UP;
        this.wristDown = DOWN;
        addRequirements(RobotContainer.wrist);
    }

    @Override
    public void execute() {
        power = (wristUp.get() - wristDown.get()) * armConstants.side;

        if(power == 0){
            if(_hold != hold){
                targetPose = wrist.getWristAngle();
                hold = true;
                _hold = hold;
            }

        }else{
            hold = false;
            wrist.setWrist(power);
        }


    }

    @Override
    public void end(boolean interrupted) {
        wrist.setWrist(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
