package frc.robot.Auto.Commands.drivebaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants.kinematics;
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
        this.xSlewRateLimiter = new SlewRateLimiter(IO.xSpeedSlewRate);
        this.ySlewRateLimiter = new SlewRateLimiter(IO.ySpeedSlewRate);
        this.turnSlewRateLimiter = new SlewRateLimiter(IO.TurnSpeedSlewRate);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double xspeed = HalfSpeed.get() ? xSpd.get()/6 : xSpd.get();
        double yspeed = HalfSpeed.get() ? ySpd.get()/6 : ySpd.get();
        double Tspeed = HalfSpeed.get() ? TSpd.get()/4 : TSpd.get();
        xspeed = MathUtil.applyDeadband(xspeed, IO.xdeadZone);
        yspeed = MathUtil.applyDeadband(yspeed, IO.ydeadZone);
        Tspeed = MathUtil.applyDeadband(Tspeed, IO.tdeadZone);
        
        if(IO.enableSlewrateLimiter){
        xspeed = xSlewRateLimiter.calculate(xspeed);
        yspeed = ySlewRateLimiter.calculate(yspeed);
        Tspeed =    turnSlewRateLimiter.calculate(Tspeed);
        }
        ChassisSpeeds chassisSpeeds;

        if (FieldOriented.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xspeed * IO.maxSpeed,
                    yspeed * IO.maxSpeed,
                    Tspeed * IO.maxTurnSpeed, 
                    swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xspeed*IO.maxSpeed, yspeed*IO.maxSpeed, Tspeed*IO.maxTurnSpeed);
        }

        //  Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = kinematics.m_kinematics.toSwerveModuleStates(chassisSpeeds);

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
