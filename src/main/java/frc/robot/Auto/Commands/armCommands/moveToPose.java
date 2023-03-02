package frc.robot.Auto.Commands.armCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.armSubsystem;

public class moveToPose extends CommandBase {
    private double targetPose, error;
    private static final armSubsystem arm = RobotContainer.arm;

    public moveToPose(double position) {
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        error = targetPose - arm.AvgPose();
        double speed =

                MathUtil.clamp(targetPose, -130, 130);
        MathUtil.clamp(
                ((error) * armConstants.proportionalGain
                        + (arm.ExtendedPose() * arm.AvgPose() * armConstants.GravGain * arm.getSide())),
                -1, 1);

        arm.manualArticulate(speed);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
