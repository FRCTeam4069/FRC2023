package frc.robot.subsystems;

/*
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDController {
    PWMSparkMax blinkinController;

    public LEDController(){
        blinkinController = new PWMSparkMax(0);
    }

    public void getValuesFromDS(){
        blinkinController.set(0);
    }

    public void changeLEDPattern(patternOptions options, boolean blueAlliance, boolean disabled){
        double pwmvalue = translateColourToNumbers(options, blueAlliance, disabled);
        
    }

    public enum patternOptions{
        YELLOW,
        PURPLE,
        RED,
        GREEN,
        BLUE,
    }

    private double translateColourToNumbers(patternOptions option, boolean blueAlliance, boolean disabled){
        double PWMValue = 0;

        if(option == patternOptions.YELLOW){
            PWMValue = 0;
        }
        else if(option == patternOptions.BLUE || (blueAlliance && disabled)){
            PWMValue = 1000;
        }
        else if(option == patternOptions.RED || (!blueAlliance && disabled)){
            PWMValue = 0;
        }
        else if(option == patternOptions.GREEN){
            PWMValue = 0;
        }
        else if(option == patternOptions.PURPLE){
            PWMValue = 0;
        }

        return PWMValue;
    }

    public void testLights(int value){
    }

}
*/