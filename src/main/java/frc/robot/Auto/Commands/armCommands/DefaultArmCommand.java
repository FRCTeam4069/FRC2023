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
    private final Supplier<Boolean> Pose1, holdPose;
    private final SlewRateLimiter speedLimiter;

    public DefaultArmCommand(armSubsystem arm, Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed, Supplier<Boolean> Pose1, Supplier<Boolean> holdPose){
 
        this.arm = arm;
        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        this.speedLimiter = new SlewRateLimiter(armConstants.speedLimiter);
        this.Pose1 = Pose1;
        this.holdPose = holdPose;

        addRequirements(arm);
    }

    @Override
    public void execute(){

        while(Pose1.get()){ 
            arm.moveToPos(0, 0.5);
        }
        while(holdPose.get()){
            arm.moveToPos(-45, 0.5);
        }
            arm.manualExtend(-extendSpeed.get());
        if(armConstants.enableSlewrateLimiter){
             arm.manualArticulate( speedLimiter.calculate(articulateSpeed.get()) );            
        }
        else{
            arm.manualArticulate(articulateSpeed.get());
        }

        SmartDashboard.putNumber("Arm Rotation Speed", articulateSpeed.get() );
        SmartDashboard.putNumber("Telescope Speed", extendSpeed.get());

       


    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
