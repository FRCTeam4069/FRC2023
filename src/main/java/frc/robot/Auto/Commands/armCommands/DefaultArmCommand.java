package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.armAndIntakeConstants.armConstants;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase{
    private final armSubsystem arm;
    private final Supplier<Double> articulateSpeed,extendSpeed ; 
    private final Supplier<Boolean> Pose1, holdPose, setZero;
    private final SlewRateLimiter speedLimiter;
    public double position = 0;

    public DefaultArmCommand(armSubsystem arm, Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed, Supplier<Boolean> Pose1, Supplier<Boolean> holdPose, Supplier<Boolean> setZero){
 
        this.arm = arm;
        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        this.speedLimiter = new SlewRateLimiter(armConstants.speedLimiter);
        this.Pose1 = Pose1;
        this.holdPose = holdPose;
        this.setZero = setZero;
        addRequirements(arm);


    }

    @Override
    public void initialize(){
        arm.setMotorPosition(position, position); // make sure the arm doesnt go crazy
    }

    @Override
    public void execute(){
        if(setZero.get()){
        position = 0;
        arm.setMotorPosition(position, position);
        }
        
        
        position += MathUtil.applyDeadband(articulateSpeed.get() * 2, 0.1);
        position = MathUtil.clamp(position, -130, 130);
        arm.moveToPos(position, 0.5);
        arm.manualExtend(extendSpeed.get());
        

        arm.joystickValues(MathUtil.applyDeadband(articulateSpeed.get(), 0.1) );

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
