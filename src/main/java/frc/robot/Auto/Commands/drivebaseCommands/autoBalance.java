package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.math.MathUtil;
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
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class autoBalance extends CommandBase {
  private final SwerveSubsystem m_drivebase = RobotContainer.swerveSubsystem;
  private final Gyro Gyro = RobotContainer.swerveSubsystem.getGyro();
  public double thetaSpeed, xspeed, balancedtime, currRoll;
  public boolean balanced,rollChange;
  public Timer timer = new Timer(); 

  private double rollThreshold = 1;
  


  public autoBalance() {
    addRequirements(RobotContainer.swerveSubsystem); // adds a requirement - if its not met then it will throw an error
  }
    @Override
    public void initialize(){
      currRoll = Gyro.getRoll();
      balanced = false;
      rollChange = false;
      SmartDashboard.putBoolean("rollChange", rollChange);
    }
    @Override
    public void execute() {
      SmartDashboard.putNumber("ROLL", currRoll-Gyro.getRoll());

      if(!rollChange){
        m_drivebase.setModuleStates(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(-2, 0, 0)));
        if(Gyro.getRoll() > currRoll+2 || Gyro.getRoll() < currRoll-2) {rollChange = true; SmartDashboard.putBoolean("rollChange", rollChange);} }
      if(!balanced && rollChange){
        xspeed = (Gyro.getRoll()-currRoll) * 0.03;
        m_drivebase.setModuleStates(kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xspeed,0 , 0)));
        if(Math.abs(currRoll-Gyro.getRoll()) < 6){
          timer.start();
          if(timer.get() == 1){balanced = true;}
        }
        else{timer.reset();}
        
      
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
      if(balanced == true) return true;  
    
      return false;
    }
}

