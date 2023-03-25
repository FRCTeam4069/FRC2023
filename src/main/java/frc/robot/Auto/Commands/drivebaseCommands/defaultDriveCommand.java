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
    private final Supplier<Boolean> HalfSpeed, turnAlign, QuarterSpeed;
    private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, turnSlewRateLimiter;

    public defaultDriveCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> TSpd, Supplier<Boolean> turnAlign,
            Supplier<Boolean> HalfSpeed, Supplier<Boolean> quarterSpeed) {

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
        double magnitude = Math.sqrt(xSpd.get() * xSpd.get() + ySpd.get() * ySpd.get());
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

        Tspeed = HalfSpeed.get() ? TSpd.get() / 4 : TSpd.get();
        Tspeed = QuarterSpeed.get() ? TSpd.get() / 6 : TSpd.get();

        if (IO.enableSlewrateLimiter) {
            Tspeed = turnSlewRateLimiter.calculate(Tspeed);
            xspeed = xSlewRateLimiter.calculate(xspeed);
            yspeed = ySlewRateLimiter.calculate(yspeed);
        }
        ChassisSpeeds chassisSpeeds;

        // Relative to field

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -yspeed * IO.maxSpeed,
                -xspeed * IO.maxSpeed,
                Tspeed * IO.maxTurnSpeed,
                swerveSubsystem.getRotation2d());

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

}