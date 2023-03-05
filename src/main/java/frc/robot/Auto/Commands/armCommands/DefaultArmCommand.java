package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase {
    private final armSubsystem arm = RobotContainer.arm;
    private final Supplier<Double> articulateSpeed, extendSpeed, side;

    public DefaultArmCommand(Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed, Supplier<Double> side) {
        this.side = side;
        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        addRequirements(RobotContainer.arm);

    }

    @Override
    public void execute() {
        // 32 for p1
        // 120 for p2
        arm.manualExtend(MathUtil.applyDeadband(-extendSpeed.get(), 0.08));
        // arm.setExtendPose(arm.extendPose+
        // MathUtil.applyDeadband(-extendSpeed.get()*1.1, 0.05));
        arm.setArmPose(arm.articulatePose + MathUtil.applyDeadband(side.get() * articulateSpeed.get() * 2, 0.08));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
