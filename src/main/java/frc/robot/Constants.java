package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {

    public static double timeOfFlight;
    public static double distToGoal;

    public class turretConstants {
        public static final int turretID = 6;
        public static final int turretEncoderID = 55;
        public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);
    }
    
    public class shooterConstants {
        public static final int shooterLeftID = 62;
        public static final int shooterRightID = 61;
        public static final int hoodID = 16;

        public static final double hoodMaxPosition = 0.015;// Maybe 0.105 //set 0.0 //0.1
        public static final double hoodMinPosition = -0.0419921875; //-0.035
        
        public static double shooterPower;  
        public static double hoodPosition;
    }
    
    public class presetShots {
        //Auto starting line, distance = 3.67m
        public static final double trenchShotRPS = 83.0; //CHECK THIS
        public static final double trenchShotHoodAngle = shooterConstants.hoodMinPosition + 0.45; //CHECK THIS
        //public static final double trenchShotTurretAngle = 0; //consider doing position to make it easier, just for the corner shot
        
        //In the corner, distance = 5.5 - 5.7m
        public static final double backCornerShotRPS = 100; //CHECK THIS
        public static final double backCornerShotHoodAngle = shooterConstants.hoodMinPosition + 0.09; //CHECK THIS
        //public static final double backCornerShotTurretAngle = 0;

        //In between the hub and the tower, distance = 2.234m
        public static final double closeShotRPS = 60.0; //CHECK THIS
        public static final double closeShotHoodAngle = shooterConstants.hoodMinPosition + 0.01338; //CHECK THIS
        //public static final double closeShotTurretAngle = 0;
    }

    public class intakeConstants {
        public static final int kickerID = 15;
        public static final int hopperID = 25;
        public static final int intakeID = 60;
        public static final int liftID = 14;

        public static final double loweredIntake = 3.4;
        public static final double upIntake = 0;
    }
    

    public class climberConstants {
        public static final int climberID = 7; // SET THIS
        public static double climberUpPosition = 0; //SET THIS
        public static double climberDownPosition = 0; //SET THIS
    }

    public class VisionConstants {
        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_BACK = "limelight-back";
        public static final String LL_RIGHT = "limelight-right";
    }
}
