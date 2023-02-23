package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase{
    private final armSubsystem arm;
    private final Supplier<Double> positivespeed,negitiveSpeed ; 
    private final SlewRateLimiter speedLimiter;

    public DefaultArmCommand(armSubsystem arm, Supplier<Double> positivespeed, Supplier<Double> negitiveSpeed){
 
        this.arm = arm;
        this.negitiveSpeed = negitiveSpeed;
        this.positivespeed = positivespeed;
        this.speedLimiter = new SlewRateLimiter(armConstants.speedLimiter);

        addRequirements(arm);
    }

    @Override
    public void execute(){
        double speed = positivespeed.get()-negitiveSpeed.get();

        if(armConstants.enableSlewrateLimiter){
             arm.manualArticulate( speedLimiter.calculate(speed) );            
        }
        else{
            arm.manualArticulate(speed);
        }

        SmartDashboard.putNumber("Arm Speed", speed );

       


    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
