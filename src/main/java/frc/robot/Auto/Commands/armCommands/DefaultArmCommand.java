package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armSubsystem;

public class DefaultArmCommand extends CommandBase{
    private final armSubsystem arm;
    private final Supplier<Double> articulateSpeed,extendSpeed, gyro; 
    private final Supplier<Boolean> Pose1, Pose2, Pose3,setZero;
    public double position = 0;

    public DefaultArmCommand(armSubsystem arm, Supplier<Double> articulateSpeed, Supplier<Double> extendSpeed, Supplier<Boolean> Pose1, Supplier<Boolean> pose2, Supplier<Boolean> pose3, Supplier<Boolean> setZero, Supplier<Double> gyro){
 
        this.arm = arm;
        this.articulateSpeed = articulateSpeed;
        this.extendSpeed = extendSpeed;
        this.Pose1 = Pose1;
        this.Pose2 = pose2;
        this.Pose3 = pose3;
        this.gyro = gyro;
        this.setZero = setZero;
        addRequirements(arm);


    }

    @Override
    public void initialize(){
        arm.setMotorPosition(position, position); // make sure the arm doesnt go crazy
    }

    public int GetSide(){
        if(Math.abs(gyro.get()) > 90){
            return -1;
        } else return 1;
    }

    @Override
    public void execute(){
        if(setZero.get()){
        position = 0;
        arm.setZero();
        }
        if(Pose1.get()){position = 0;}
        if(Pose2.get()){position = 55 * -GetSide();}
        if(Pose3.get()){position = 130 * -GetSide();}
        
        position += MathUtil.applyDeadband(articulateSpeed.get() * 2, 0.1);
        position = MathUtil.clamp(position, -140, 140);
        arm.moveToPos(position, 1);
        arm.manualExtend(extendSpeed.get()*0.75);
        

        arm.joystickValues(MathUtil.applyDeadband(articulateSpeed.get(), 0.1) );

        SmartDashboard.putNumber("Target Position", position);
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
