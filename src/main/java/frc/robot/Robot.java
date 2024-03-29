// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  private static final String Halo = "Halo.chrp";
  private static final String PotC = "PiratesOfTheCaribbean.chrp";
  private static final String StarWars = "StarWars.chrp";
  private static final String IWITW = "IWantItThatWay.chrp";
  private static final String tetris = "tetrisTheme.chrp";
  private String musicfile;
  private final SendableChooser<String> Music_Chooser = new SendableChooser<>();

  //public LEDController led;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight.local", port);
  }

    m_robotContainer = new RobotContainer();

    Music_Chooser.setDefaultOption("Pirates Of The Caribbean", PotC);
    Music_Chooser.addOption("Halo", Halo);
    Music_Chooser.addOption("I Want It That Way", IWITW);
    Music_Chooser.addOption("Tetris Theme", tetris);
    Music_Chooser.addOption("Star Wars", StarWars);
    SmartDashboard.putData(Music_Chooser);
    //led = new LEDController();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {


  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    //Set FF controllers 
    RobotContainer.swerveSubsystem.FRSwerveModule.setCurrentFFController(0);
    RobotContainer.swerveSubsystem.FLSwerveModule.setCurrentFFController(0);
    RobotContainer.swerveSubsystem.BRSwerveModule.setCurrentFFController(0);
    RobotContainer.swerveSubsystem.BLSwerveModule.setCurrentFFController(0);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //Set FF controllers 
    RobotContainer.swerveSubsystem.FRSwerveModule.setCurrentFFController(1);
    RobotContainer.swerveSubsystem.FLSwerveModule.setCurrentFFController(1);
    RobotContainer.swerveSubsystem.BRSwerveModule.setCurrentFFController(1);
    RobotContainer.swerveSubsystem.BLSwerveModule.setCurrentFFController(1);
    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    // musicfile = Music_Chooser.getSelected();
    // m_robotContainer.PlayMusic(musicfile);
    // SmartDashboard.putString("Selected file :", musicfile);

    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
  }

}
