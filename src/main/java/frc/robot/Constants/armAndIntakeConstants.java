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

        //for Arm move to position
        public static final double proportionalGain = 0.02;
        public static final double GravGain = -0.0001; // set to 0 to disable 

        public static final float softlimits = 130;
        public static double side;


        public static double armPose;
        public static double extendPose;

        // For forward kinematics
        public static final double L1 = 24; // inches, base of robot to shoulder
        public static double L2; // inches, L2_OFFSET + Extension amount
        public static final double L2_OFFSET = 5; // inches + extension amount
        public static final double L3 = 12; // inches, wrist length


        public static double c1 = 0, s1 = 1; // 1 since J1 does not move and is 90
        public static double c2, s2, c3, s3;
        public static double x = 0, y = 0; // x is away from the center of the robot, y is up

        public static final double XLIMIT = 48;
        public static final double YLIMIT = 54;

        // formula for arm stay to stay at pose
        // speed = input + armlength * kP + armAgle * kP 
    

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
        
        /*
         * Extender Motor
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

        public static final int INTAKE_ID = 13;
        public static final int WRIST_ID = 12;
        public static final double wristkP = 0.8;
        public static double wristPose;
        public static final double intakekP = 0.3;

        /*
         * degrees/tick
         * 
         */

    }
    
        
    

    


}