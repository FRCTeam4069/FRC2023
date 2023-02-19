package frc.robot.subsystems.SubsystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DefualtDriveCommand extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpd, ySpd, TSpd;
    private final Supplier<Boolean> FieldOriented;
    private final Supplier<Boolean> HalfSpeed;
    private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, turnSlewRateLimiter;

    public DefualtDriveCommand(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> TSpd, 
    Supplier<Boolean> FieldOriented, Supplier<Boolean> HalfSpeed) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpd = xSpd;
        this.ySpd = ySpd;
        this.TSpd = TSpd;
        this.HalfSpeed = HalfSpeed;
        this.FieldOriented = FieldOriented;
        this.xSlewRateLimiter = new SlewRateLimiter(InputConstants.xSpeedSlewRate);
        this.ySlewRateLimiter = new SlewRateLimiter(InputConstants.ySpeedSlewRate);
        this.turnSlewRateLimiter = new SlewRateLimiter(InputConstants.TurnSpeedSlewRate);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double xspeed = HalfSpeed.get() ? xSpd.get()/6 : xSpd.get();
        double yspeed = HalfSpeed.get() ? ySpd.get()/6 : ySpd.get();
        double Tspeed = HalfSpeed.get() ? TSpd.get()/4 : TSpd.get();

        xspeed = Math.abs(xspeed)>InputConstants.xdeadZone? xspeed : 0;
        yspeed = Math.abs(yspeed)>InputConstants.ydeadZone? yspeed : 0;
        Tspeed = Math.abs(Tspeed)>InputConstants.tdeadZone? Tspeed : 0;
        
        if(Constants.enableSlewrateLimiter){
        xspeed = xSlewRateLimiter.calculate(xspeed);
        yspeed = ySlewRateLimiter.calculate(yspeed);
        Tspeed =    turnSlewRateLimiter.calculate(Tspeed);
        }
        ChassisSpeeds chassisSpeeds;

        if (FieldOriented.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xspeed * Constants.InputConstants.maxSpeed,
                    yspeed * Constants.InputConstants.maxSpeed,
                    Tspeed * Constants.InputConstants.maxTurnSpeed, 
                    swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xspeed*Constants.InputConstants.maxSpeed, yspeed*Constants.InputConstants.maxSpeed, Tspeed*Constants.InputConstants.maxTurnSpeed);
        }

        //  Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DrivebaseConstants.m_kinematics.toSwerveModuleStates(chassisSpeeds);

        //  Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
