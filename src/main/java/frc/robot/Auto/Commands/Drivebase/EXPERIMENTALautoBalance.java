package frc.robot.Auto.Commands.Drivebase;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class EXPERIMENTALautoBalance extends CommandBase {
  private final SwerveSubsystem m_drivebase;
  private final Supplier<Boolean> autoBalanceON;


  public EXPERIMENTALautoBalance(SwerveSubsystem drivebase, Supplier<Boolean> isActive) {
    this.m_drivebase = drivebase;
    this.autoBalanceON = isActive;

    addRequirements(drivebase); // adds a requirement - if its not met then it will throw an error
  }

    @Override
    public void execute() {

    // what to do while this command is active
    
      /*
       * 1. get current angle of the robot (or what direction the ramp is located )
       * 2. From that figure out if you need to use the pitch or roll, or need to do the math for differnt angle of "climb"
       * 3. tell robot to go forward at a set speed until a change in pitch or roll occurs
       * 4. using that pitch or roll change as an error value for a pid loop gett the robot to balance 
       */
    SmartDashboard.putBoolean("Auto Balance", autoBalanceON.get());


  }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
        m_drivebase.stopModules(); // stops wheels
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

