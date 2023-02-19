package frc.robot.Auto.Commands.drivebaseCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class autoBalance extends CommandBase {
  private final SwerveSubsystem m_drivebase;
  private final Gyro Gyro;
  public double thetaSpeed, xspeed;
  public PIDController xController = new PIDController( 0.05, 0,0);
  public PIDController rotController = new PIDController( 0.05, 0,0);
  public boolean balanced;
  private double pitchThreshold = 1;
  


  public autoBalance(SwerveSubsystem drivebase, Gyro gyro) {
    this.m_drivebase = drivebase;
    this.Gyro = gyro;

    addRequirements(drivebase); // adds a requirement - if its not met then it will throw an error
  }

    @Override
    public void execute() {
      SwerveModuleState[] states;
      boolean PitchChange = false;
      balanced = false;
      double currPitch = Gyro.getPitch();

      while(!PitchChange){
        states = kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(2, 0, 0));
        m_drivebase.setModuleStates(states);

        if(Gyro.getPitch() > currPitch+1|| Gyro.getPitch() < currPitch-1) PitchChange = true;

      }
      if(PitchChange){
      while(!balanced){
        xspeed = xController.calculate(Gyro.getPitch());
        Gyro.resetGyro();
        thetaSpeed = Gyro.getYaw() * 0.1;
        states = kinematics.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xspeed, 0, thetaSpeed));
        m_drivebase.setModuleStates(states);

        if(Math.abs(Gyro.getPitch()) < pitchThreshold) balanced = true; 
        SmartDashboard.putBoolean("Balanced", balanced);


      }
    }

    

  }

    @Override
    public void end(boolean interrupted) {
        // what to do at the end
        m_drivebase.stopModules(); // stops wheels
    }

    @Override
    public boolean isFinished() {
      if(balanced == true) return true;  
    
      return false;
    }
}

