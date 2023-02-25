package frc.robot.Constants;

public class armAndIntakeConstants{




    public static class armConstants{
        public static final int ARM_ID_L = 15; // Left
        public static final int ARM_ID_R = 4; // Right
        public static final int ARM_ID_E = 5; // Extend //FIXME
        public static final boolean rightMotorInvert = false; 
        public static final boolean leftMotorInvert = true; 
        public static final boolean telescopeMotorInvert = false; 

        public static final boolean enableSlewrateLimiter = true;
        public static final double speedLimiter = 1;

        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        public static final double kP = 0;
        public static final double kD = 0;

        /* Articulate motor gears */
        // In  : Out
        // 20  : 1    - motor
        // 24  : 78   - gear
        // 15  : 42   - Sprocket
        /*  Overall ratios:
         * -> 20   : 1
         * -> 3.25 : 1
         * -> 2.8  : 1
         * -> 182  : 1
         * 
         * Encoders conversion:
         * -> 42 : 1        - 42 counts / motor rev
         * -> 182 : 1       - 182 motor rev / arm rev
         * -> 1 : 360       - arm rev / 360 degrees
         * 360 / ( 182 * 42 )
         * 
        */
        
        /* Extender Motor
         * 4 to 1 
         * 18 to 42 
         * 
         * Overall 
         * 72 : 42 
         * 72/42 : 1
         * 1.7148571 * 42 encoder counts/ motor rev
         * 72 ticks per 0.5in out/in
         * 
         *  
         * 168 counts per rev 
         * 18 to 42 gear 
         * 2 1/3 : 1
         * 392.00004 : 1 
         * 
         * 392 counts per 0.5in in/out
         *
         */

    }

    public static class intakeConstants{

        public static final int INTAKE_ID = 0;
        public static final int ENCODER_ID_A = 0;
        public static final int ENCODER_ID_B = 0;
        public static final int LIMIT_SWITCH_ID_2 = 0;
        public static final int LIMIT_SWITCH_ID_1 = 0;

    }
    
        
    

    


}