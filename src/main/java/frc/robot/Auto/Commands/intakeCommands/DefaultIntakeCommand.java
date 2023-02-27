package frc.robot.Auto.Commands.intakeCommands;

import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase{

    private final Intake intake;
    private final Supplier<Boolean> intakeOpen, intakeClose;
    private final Supplier<Double> wristUp, wristDown;
    public double intakeSpeed;
   
    /**
     * 
     * @param intake
     * @param UP
     * @param DOWN
     * @param OPEN
     * @param CLOSE
     */
    public DefaultIntakeCommand(Intake intake, Supplier<Double> UP, Supplier<Double> DOWN, Supplier<Boolean> OPEN, Supplier<Boolean> CLOSE) {
        this.intake = intake;

        this.wristUp = UP; 
        this.wristDown = DOWN;
        this.intakeOpen = OPEN;
        this.intakeClose = CLOSE;

        addRequirements(intake);
    }

    @Override
    public void execute(){
        
        intake.setWrist(wristUp.get() - wristDown.get());
        SmartDashboard.putNumber("intake Speed", wristUp.get() - wristDown.get());
        
        if(intakeOpen.get() && intakeClose.get()){
            intakeSpeed = 0;
        }else if(intakeClose.get()){
            intakeSpeed = -1;
        }else if(intakeOpen.get()){
            intakeSpeed = 1;
        }else{
            intakeSpeed = 0;
        }

        intake.setIntake(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted){
        intake.setIntake(0);
        intake.setWrist(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
