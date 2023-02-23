package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase{
    private final armSubsystem arm;
    private final Supplier<Double> articulateSpeed,extendSpeed ; 
    private final SlewRateLimiter speedLimiter;

    public DefaultArmCommand(armSubsystem arm, Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed){
 
        this.arm = arm;
        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        this.speedLimiter = new SlewRateLimiter(armConstants.speedLimiter);
       

        addRequirements(arm);
    }

    @Override
    public void execute(){
            arm.manualExtend(-extendSpeed.get());
        if(armConstants.enableSlewrateLimiter){
             arm.manualArticulate( speedLimiter.calculate(articulateSpeed.get()) );            
        }

        else{
            arm.manualArticulate(articulateSpeed.get());
        }

        SmartDashboard.putNumber("Arm Rotation Speed", articulateSpeed.get() );
        SmartDashboard.putNumber("Telescope Speed", extendSpeed.get() );

       


    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
