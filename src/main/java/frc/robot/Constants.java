package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {

    public static double timeOfFlight;

    // public InterpolatingDoubleTreeMap finalHoodPosition = new InterpolatingDoubleTreeMap();
    // public InterpolatingDoubleTreeMap finalShooterRPS = new InterpolatingDoubleTreeMap();
    // public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

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

        public static final double hoodMaxPosition = 0.105;// Maybe 0.105 //set 0.0 //0.1
        public static final double hoodMinPosition = hoodMaxPosition - 0.135; //-0.035
        
        public static double shooterPower;  
        public static double hoodPosition;

        
    }
    
    public class presetShots {
        public static final double trenchShotRPM = 100.0;
        public static final double trenchShotHoodAngle = 0.0;
        //public static final double trenchShotTurretAngle = 0; //consider doing position to make it easier, just for the corner shot

        public static final double cornerShotRPM = 0; //SET THIS
        public static final double cornerShotHoodAngle = 0.0; //SET THIS
        //public static final double cornerShotTurretAngle = 0;

        public static final double closeShotRPM = 0; //SET THIS
        public static final double closeShotHoodAngle = 0.0; //SET THIS
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
        public static final int climberID = 0; // SET THIS
        public static double climberUpPosition = 0; //SET THIS
        public static double climberDownPosition = 0; //SET THIS
    }

    public class VisionConstants {
        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_BACK = "limelight-back";
        public static final String LL_RIGHT = "limelight-right";
    }
}
