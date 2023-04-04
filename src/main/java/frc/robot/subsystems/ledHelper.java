package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledHelper extends SubsystemBase {

   private PWM blinkin;
   public Colours defColor = Colours.YELLOW;

   public ledHelper(int PWMport) {
      blinkin = new PWM(PWMport);
   }

   public void ChangePWM(Colours colours) {
      blinkin.setRaw(TranslateColour(colours));
   }

   private int TranslateColour(Colours colour) {
      if (colour == Colours.RED)
         return red;
      if (colour == Colours.BLUE)
         return blue;
      if (colour == Colours.STARTUPPATTERN)
         return StartUpPatern;
      if (colour == Colours.YELLOW)
         return yellow;
      if (colour == Colours.PURPLE)
         return purple;
      if (colour == Colours.GREEN)
         return green;
      if (colour == Colours.PURPLEFLASHING)
         return FlashingPrple;
      if (colour == Colours.BlueFlashing)
         return FlashingBlue;
      if (colour == Colours.ERROR_YELLOw)
         return ErrorYellow;
      if (colour == Colours.GREEN_WHITE_PATTERN)
         return greenAndWhitePattern;
      if (colour == Colours.ERROR_RED)
         return ErroRed;
      if (colour == Colours.COOL_PATTERN)
         return CoolPatern;
      else
         return 0;
   }

   private final int red = 1300;
   private final int blue = 800;
   private final int StartUpPatern = 33600;
   private final int yellow = 9700;
   private final int purple = 21600;
   private final int green = 42350;
   private final int FlashingPrple = 35350;
   private final int FlashingBlue = 5050;
   private final int ErrorYellow = 17350;
   private final int greenAndWhitePattern = 17200;
   private final int ErroRed = 46000;
   private final int CoolPatern = 53950;

   public enum Colours {
      RED,
      BLUE,
      STARTUPPATTERN,
      YELLOW,
      PURPLE,
      GREEN,
      PURPLEFLASHING,
      BlueFlashing,
      ERROR_YELLOw,
      ERROR_RED,
      GREEN_WHITE_PATTERN,
      COOL_PATTERN
   }

   public CommandBase setDefaultColor(Colours color){
      return this.runOnce(() -> defColor = color);

   }

}
