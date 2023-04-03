package frc.robot.Auto.Commands.drivebaseCommands;

import java.util.function.Supplier;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.swerveSubsystem;
import frc.robot.subsystems.cameraHelper;

public class defaultDriveCommand extends CommandBase {
    private final swerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpd, ySpd, TSpd;
    private final Supplier<Boolean> HalfSpeed, turnAlign, QuarterSpeed;
    private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, turnSlewRateLimiter;
    private final cameraHelper f_LimeLight = RobotContainer.frontLimeLight;
    private final cameraHelper b_LimeLight = RobotContainer.backLimeLight;
    private cameraHelper cameraToUse;

    private PIDController turninPID = new PIDController(0.07, 0, 0);

    public defaultDriveCommand(swerveSubsystem swerveSubsystem,
            Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> TSpd, Supplier<Boolean> turnAlign,
            Supplier<Boolean> HalfSpeed, Supplier<Boolean> quarterSpeed) {

        turninPID.enableContinuousInput(-180, 180);

        this.swerveSubsystem = swerveSubsystem;
        this.xSpd = xSpd;
        this.turnAlign = turnAlign;
        this.QuarterSpeed = quarterSpeed;
        this.ySpd = ySpd;
        this.TSpd = TSpd;
        this.HalfSpeed = HalfSpeed;
        this.xSlewRateLimiter = new SlewRateLimiter(IO.xSpeedSlewRate);
        this.ySlewRateLimiter = new SlewRateLimiter(IO.ySpeedSlewRate);
        this.turnSlewRateLimiter = new SlewRateLimiter(IO.TurnSpeedSlewRate);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        updateUsedCamera();

        f_LimeLight.printerNumbers();
        b_LimeLight.printerNumbers();
        SmartDashboard.putNumber("Closest 180", turnToClosest());
        f_LimeLight.setMode(0);
        b_LimeLight.setMode(0);
        double Tspeed;
        double xspeed;
        double yspeed;
        xspeed = MathUtil.applyDeadband(xSpd.get(), IO.xdeadZone);
        yspeed = MathUtil.applyDeadband(ySpd.get(), IO.ydeadZone);
        Tspeed = MathUtil.applyDeadband(TSpd.get(), IO.tdeadZone);

        if (QuarterSpeed.get()) {
            xspeed = xspeed / 8;
            yspeed = yspeed / 8;
        } else if (HalfSpeed.get()) {
            xspeed = xspeed / 6;
            yspeed = yspeed / 6;
        }

        Tspeed = HalfSpeed.get() ? Tspeed / 4 : Tspeed;
        Tspeed = QuarterSpeed.get() ? Tspeed / 6 : Tspeed;

        if (IO.enableSlewrateLimiter) {
            Tspeed = turnSlewRateLimiter.calculate(Tspeed);
            xspeed = xSlewRateLimiter.calculate(xspeed);
            yspeed = ySlewRateLimiter.calculate(yspeed);
        }
        ChassisSpeeds chassisSpeeds;

        if(turnAlign.get()){
            f_LimeLight.setLED(3);
            b_LimeLight.setLED(3);
            b_LimeLight.setPipeline(0);
            f_LimeLight.setPipeline(0);
            SmartDashboard.putNumber("Camera To Use", cameraToUse.tv());
            turninPID.setSetpoint(turnToClosest());
            if(cameraToUse.tv() == 1 && turnedTo(turnToClosest())){
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    -(0.38 - cameraToUse.ta()) * 4,
                dynamicSpeedY(),
                MathUtil.applyDeadband( -turninPID.calculate(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()), 0.1), //* IO.maxTurnSpeed,
                swerveSubsystem.getRotation2d());

            } else{
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -yspeed * IO.maxSpeed,
                -xspeed * IO.maxSpeed,
                MathUtil.applyDeadband( -turninPID.calculate(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()), 0.1), //* IO.maxTurnSpeed,
                swerveSubsystem.getRotation2d());
            }

        }else{
            f_LimeLight.setLED(1);
            b_LimeLight.setLED(1);
            b_LimeLight.setPipeline(1);
            f_LimeLight.setPipeline(1);
          
            
        //Relative to field

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -yspeed * IO.maxSpeed,
                -xspeed * IO.maxSpeed,
                Tspeed * IO.maxTurnSpeed,
                swerveSubsystem.getRotation2d());
        }
        

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kinematics.m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // Output each module states to wheels
        swerveSubsystem.setModuleState(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void updateUsedCamera(){
        if(swerveSubsystem.getSide() == 1){
            cameraToUse = b_LimeLight;
        }else{
            cameraToUse = f_LimeLight;
        }
    }

    public double turnToClosest(){
        if(Math.abs(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees()) > 90 ){
            return 180;
        }
        else return 0;

    }

    public double dynamicSpeedY(){
        double error =((-2.5) - cameraToUse.ty());
        double deadband;
        boolean atPOse = false;
        if(Math.abs(error) > 0.1){
            deadband = 0.2;
        }else{
            deadband = 0.05;
        }

        return MathUtil.applyDeadband(error * 0.1, deadband);

    }

    public boolean turnedTo(double angle){
        double turningError = 
        -Math.abs(swerveSubsystem.odometry.getPoseMeters().getRotation().getDegrees())
        + (Math.abs(angle));
        
        if(turningError < 5){
            return true;
        }
        else{
            return false;
        }

    }

   

}