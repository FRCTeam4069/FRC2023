package frc.robot.Auto.Commands.Drivebase;

import java.sql.Driver;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class EXPERIMENTALautoBalance extends CommandBase {
  private final SwerveSubsystem m_drivebase;
  private final Gyro Gyro;
  public double xspeed;
  public PIDController xController;


  public EXPERIMENTALautoBalance(SwerveSubsystem drivebase, Gyro gyro) {
    this.m_drivebase = drivebase;
    this.Gyro = gyro;

    addRequirements(drivebase); // adds a requirement - if its not met then it will throw an error
  }

    @Override
    public void execute() {
      SwerveModuleState[] states;
      boolean PitchChange = false, balanced = false;
      double currPitch = Gyro.getPitch();

      while(!PitchChange){
        states = DrivebaseConstants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.2, 0, 0));
        m_drivebase.setModuleStates(states);

        if(Gyro.getPitch() > currPitch+1|| Gyro.getPitch() < currPitch-1) PitchChange = true;

      }
      if(PitchChange){
      while(!balanced){
        xspeed = xController.calculate(Gyro.getPitch());
        states = DrivebaseConstants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xspeed, 0, 0));
        m_drivebase.setModuleStates(states);

        if(Math.abs(Gyro.getPitch()) < 1) balanced = true;


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
        return false;
    }
}

