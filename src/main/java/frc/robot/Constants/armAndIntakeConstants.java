package frc.robot.Constants;

public class armAndIntakeConstants{




    public static class armConstants{
        public static final int ARM_ID_L = 15; // Left
        public static final int ARM_ID_R = 4; // Right
        public static final int ARM_ID_E = 0; // Extend //FIXME
        public static final boolean rightMotorInvert = false; 
        public static final boolean leftMotorInvert = true; 

        public static final boolean enableSlewrateLimiter = true;
        public static final double speedLimiter = 1;

        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        public static final double kP = 0;
        public static final double kD = 0;

    }

    public static class intakeConstants{

        public static final int INTAKE_ID = 0;
        public static final int ENCODER_ID_A = 0;
        public static final int ENCODER_ID_B = 0;
        public static final int LIMIT_SWITCH_ID_2 = 0;
        public static final int LIMIT_SWITCH_ID_1 = 0;

    }
    
        
    

    


}