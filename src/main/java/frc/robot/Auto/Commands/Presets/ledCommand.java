package frc.robot.Auto.Commands.Presets;

import java.util.function.Supplier;

import org.opencv.photo.AlignExposures;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ledHelper;
import frc.robot.subsystems.ledHelper.Colours;

public class ledCommand extends CommandBase {
    private final ledHelper leds = RobotContainer.leds;
    public int allianceColor;
    private final Supplier<Boolean> hasSomething, autoAligning;
    public ledCommand(Supplier<Boolean> hasSomething, Supplier<Boolean> autoAlinging) {
        this.hasSomething = hasSomething;
        this.autoAligning = autoAlinging;
        addRequirements(RobotContainer.leds);
    }
    @Override
    public void execute(){
        if(hasSomething.get()) leds.ChangePWM(Colours.GREEN);
        else if(!hasSomething.get()) leds.ChangePWM(leds.defColor);
        else if(autoAligning.get()) leds.ChangePWM(Colours.YELLOW);

    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
