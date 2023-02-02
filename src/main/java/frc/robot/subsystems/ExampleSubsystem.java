package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    // initialize things here 
    private double ExamplePosition;

    public ExampleSubsystem(){
        ExamplePosition = 0;
    }

    public double ExampleOutput(){
        return ExamplePosition;
    }

    public void ExampleInput(double ExamplePosition){
        this.ExamplePosition += ExamplePosition;
    }
}
