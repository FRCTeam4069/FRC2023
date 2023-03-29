package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as
 * {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command
 * this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class autoBalance extends CommandBase {
  private final SwerveSubsystem m_drivebase = RobotContainer.swerveSubsystem;
  private final Gyro Gyro = RobotContainer.swerveSubsystem.getGyro();
  private final double initSpeed;
  public double thetaSpeed, xspeed, balancedtime, currpitch;
  public final Double Timeout; 
  public boolean balanced, pitchChange;
  private final boolean reversed;
  public PIDController PIDcontrol = new PIDController(0.03, 0.0001, 0);
  public Timer timer = new Timer(), timeOutTimer = new Timer();
  /**
   * 
   * @param Timeout Seconds until command times out.
   */
  public autoBalance(double Timeout, boolean reversed, double initSpeed) {
    this.Timeout = Timeout;
    this.reversed = reversed;
    this.initSpeed = initSpeed;
    addRequirements(RobotContainer.swerveSubsystem); // adds a requirement - if its not met then it will throw an error
  }

  @Override
  public void initialize() {
    timeOutTimer.reset();
    timeOutTimer.stop();
    currpitch = Gyro.getPitch();
    balanced = false;
    pitchChange = false;
    PIDcontrol.setSetpoint(currpitch);
    m_drivebase.setModuleState(m_drivebase.angleModules(0));
 
  }

  @Override
  public void execute() {
    
    timeOutTimer.start();   

    if (!pitchChange) {
      m_drivebase.setModuleState(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(.4 * (reversed ? -1 : 1), 0, 0)));
      if (Math.abs(Gyro.getPitch() - currpitch) > 15) {
        pitchChange = true;
        SmartDashboard.putBoolean("pitchChange", pitchChange);
      }
    }
    if (!balanced && pitchChange) {
      xspeed = MathUtil.clamp(PIDcontrol.calculate(-(Gyro.getPitch() - currpitch)), -0.25, 0.25);
      if(xspeed * (reversed ? -1 : 1) < 0){ PIDcontrol.setP(0.017);}
      m_drivebase.setModuleState(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xspeed, 0, 0)));
      if (Math.abs(currpitch - Gyro.getPitch()) < 6) {
        timer.start();
        if (timer.get() == 1) {
          System.out.println("Balanced = True");
          balanced = true;
        }
      } else {
        timer.reset();
      }

    }

  }

  @Override
  public void end(boolean interrupted) {
    // what to do at the end
    m_drivebase.stopModules(); // stops wheels
    balanced = true;
  }

  @Override
  public boolean isFinished() {
    if (balanced == true || timeOutTimer.hasElapsed(Timeout))
      return true;
    else {
      return false;
    }
  }
}
