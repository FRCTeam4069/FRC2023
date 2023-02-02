package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class ExampleCommand extends CommandBase {
  private final ExampleSubsystem m_ExampleSubsystem;
  private final BooleanSupplier m_ExampleInput1;
  private final BooleanSupplier m_ExampleInput2;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public ExampleCommand(ExampleSubsystem ExampleSubsystem, BooleanSupplier ExampleInput1, BooleanSupplier ExampleInput2) {
    m_ExampleSubsystem = ExampleSubsystem;
    m_ExampleInput1 = ExampleInput1;
    m_ExampleInput2 = ExampleInput2;

    addRequirements(ExampleSubsystem); // adds a requirement - if its not met then it will throw an error
  }

    @Override
    public void execute() {
    // what to do while this command is active
    // this example will add or subtract 5 depenting on the input and print the current value

    m_ExampleSubsystem.ExampleInput( m_ExampleInput1.getAsBoolean() ? 5 : 0);
    m_ExampleSubsystem.ExampleInput( m_ExampleInput2.getAsBoolean() ? -5 : 0);

    SmartDashboard.putNumber("Example Output", m_ExampleSubsystem.ExampleOutput());
  }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
        // this example sets the ExampleSubsytems double ExamplePosition to 0
        m_ExampleSubsystem.ExampleInput( - m_ExampleSubsystem.ExampleOutput());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

