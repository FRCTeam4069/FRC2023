package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class FieldOrientedMTP extends CommandBase {
    private final double targetPose, threshold;
    private Supplier<Double> side;
    private static final armSubsystem arm = RobotContainer.arm;

    public FieldOrientedMTP(double position, double threshold, Supplier<Double> robotSide) {
        this.targetPose = position;
        this.side = robotSide;
        this.threshold = threshold;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        arm.setArmPose(targetPose * side.get());
        SmartDashboard.putNumber("SIDE", side.get());

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(threshold != -1){
        return arm.isAtPoseAT(targetPose, threshold);
        } else return false;

    }

}

