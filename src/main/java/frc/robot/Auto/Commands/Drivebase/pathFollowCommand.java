package frc.robot.Auto.Commands.Drivebase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class pathFollowCommand extends CommandBase {

  public pathFollowCommand() {
  

  }

    @Override
    public void execute() {
    // what to do while this command is active
  }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

