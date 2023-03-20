package frc.robot.Auto.Commands.drivebaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IO;
import frc.robot.Constants.drivebaseConstants.kinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class defaultDriveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpd, ySpd, TSpd;
    private final Supplier<Boolean> HalfSpeed, turnAlign;
    private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, turnSlewRateLimiter;

    public defaultDriveCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> TSpd, Supplier<Boolean> turnAlign,
            Supplier<Boolean> HalfSpeed) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpd = xSpd;
        this.turnAlign = turnAlign;
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
        double Tspeed;
        double xspeed = HalfSpeed.get() ? xSpd.get() / 6 : xSpd.get();
        double yspeed = HalfSpeed.get() ? ySpd.get() / 6 : ySpd.get();
        xspeed = MathUtil.applyDeadband(xspeed, IO.xdeadZone);
        yspeed = MathUtil.applyDeadband(yspeed, IO.ydeadZone);
        SmartDashboard.putNumber("heading", swerveSubsystem.getGyro().getHeading());
        SmartDashboard.putNumber("Sub 90", 90 - swerveSubsystem.getGyro().getHeading());
        SmartDashboard.putNumber("Sub 180", 270 - swerveSubsystem.getGyro().getHeading());

        if (IO.enableSlewrateLimiter) {
            xspeed = xSlewRateLimiter.calculate(xspeed);
            yspeed = ySlewRateLimiter.calculate(yspeed);
        }
        ChassisSpeeds chassisSpeeds;

        // Relative to field
        if (turnAlign.get()) {
            if (swerveSubsystem.getSide() == 1) {
                Tspeed = MathUtil.clamp(
                        0.1 * -(90 - Math.abs(swerveSubsystem.getGyro().getHeading())), -2, 2);
            } else {
                Tspeed = MathUtil.clamp(
                        0.1 * -(270 - Math.abs(swerveSubsystem.getGyro().getHeading())), -2, 2);
            }
        } else {
            Tspeed = HalfSpeed.get() ? TSpd.get() / 4 : TSpd.get();
            Tspeed = MathUtil.applyDeadband(Tspeed, IO.tdeadZone);
            Tspeed = turnSlewRateLimiter.calculate(Tspeed);
        }
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                yspeed * IO.maxSpeed,
                xspeed * IO.maxSpeed,
                Tspeed * IO.maxTurnSpeed,
                swerveSubsystem.getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kinematics.m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // Output each module states to wheels
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
