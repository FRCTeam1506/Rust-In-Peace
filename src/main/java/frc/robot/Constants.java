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
    public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);


    public class SwerveConstants {

        public static final double driveKP = 1.5;
        public static final double driveKI = 0.15; //0
        public static final double driveKD = 0.175;//0.075

        public static final double alignKP = 1.5; //1.5
        public static final double alignKI = 0.15;//0
        public static final double alignKD = 0.175;//0.075
        public static final double dMaxVelocity = 1;
        public static final double dMaxAccel = 2;

        public static final double tMaxVelocity = 4; //rad/s
        public static final double tMaxAccel = 1;

        //on the fly path constraitns
        public static PathConstraints oTF_Constraints = new PathConstraints(5.3, 5, Math.toRadians(270), Math.toRadians(360));    
    }

    public class turretConstants {
        public static final int turretID = 6;
        public static final int turretEncoderID = 55;
    }
    public class shooterConstants {
        public static final int shooterLeft = 62;
        public static final int shooterRight = 61;
        public static final int hood = 16;
        public static final double hoodMaxPosition = -3;//set
        public static final double hoodMinPosition = -1.5;
        public static final double wheelDiameterMeters = 0.0762;
        public static final double kSlip = 0.85; //HOW MUCH THE WHEELS SLIP
        

        public static double shooterPower;  
        public static double hoodPosition;
        public static double defaultHoodPosition = -0.6;

    }

    public class intakeConstants {
        public static final int kickerID = 15;
        public static final int hopperID = 25;
        public static final int intakeID = 60;
        public static final int liftID = 14;

        public static final double loweredIntake = 3.822754;
        public static final double upIntake = 0;
    }
    

    public class climberConstants {
        public static final int climberID = 0; // SET THIS
        public static double climberUpPosition;
        public static double climberDownPosition;

    }

    public class calculatedShotConstants {
        public static double tX; //Hub X
        public static double tY; //Hub Y

    }

    public class VisionConstants {

        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_RIGHT = "limelight-right";
        //public static final String LL_FRONT = "limelight-front"; //human player side

        // public static final double coralStationLeftHeading = 235;
        // public static final double coralStationRightHeading = 125;   
        
        
        //todo: definitely change these values for our robot.
        // X is in the normal direction of the tag, Y is parallel to the tag 
        public static final Transform2d leftBranch = new Transform2d(0.23769, -0.35, new Rotation2d(Math.toRadians(-2.8))); // 0.237 x perfect for left branch after states // -0.46769 x, //math.pi puts the ramp touching the reef //-4 DEGREES good at FRCC, but too angled for SVSU hemlock, reduced to -2.8.
        public static final Transform2d rightBranch = new Transform2d(0.23769, 0.0, new Rotation2d(Math.toRadians(-2.6))); //both used to be 0 degrees, but -4 is (italian chef kiss)
        public static final Transform2d reefAlgae = new Transform2d(0.5,0.0,new Rotation2d(0));


        public static double std02 = 15; //standard deviation for vision??? seems high
        public static double maxStdDeviation = .5; 
        /** cut-off tag area, don't trust past this point */
        public static final double minTagArea = 0.35; 
        public static double visionStdSlope = (maxStdDeviation-std02)/(4-.2); // from .2 to 4 // 2m to .5m // units (Deviation / Tag Area)
        public static double visionStdConstant = std02 - visionStdSlope * .2; // Units (Deviation)
    
        public static double getVisionStd(double tagArea){
            double std = visionStdSlope * tagArea + visionStdConstant;
    
            if (tagArea < minTagArea){ 
                return 9999999;
            }
            if (std < maxStdDeviation){
                return maxStdDeviation;
            }
            
            return std;
        }
    }
}
