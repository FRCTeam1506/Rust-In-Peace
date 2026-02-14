package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {

    public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);

    public class turretConstants {
        public static final int turretID = 6;
    }
    public class shooterConstants {
        public static final int shooterLeft = 62;
        public static final int shooterRight = 61;
        public static final int hood = 16;
        public static final double hoodUpPosition = -3;
        public static final double hoodDownPosition = -2;
    }

    public class intakeConstants {
        public static final int kickerID = 15;
        public static final int hopperID = 25;
        public static final int intakeID = 60;
        public static final int liftID = 14;

        public static final double loweredIntake = 3.75;
        public static final double upIntake = 0.25;
    }
    
}
