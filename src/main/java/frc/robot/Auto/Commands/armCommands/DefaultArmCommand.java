package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase {
    private final armSubsystem arm = RobotContainer.arm;
    private final Supplier<Double> articulateSpeed, extendSpeed;

    public DefaultArmCommand(Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed) {

        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        addRequirements(RobotContainer.arm);

    }

    @Override
    public void execute() {

        arm.manualExtend(MathUtil.applyDeadband( extendSpeed.get(), 0.08));
        arm.setArmPose(arm.articulatePose + MathUtil.applyDeadband(articulateSpeed.get()*2, 0.08) );
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
