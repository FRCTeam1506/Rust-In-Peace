package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.LimelightHelpers;
// import frc.robot.util.reefData;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Field Widget in Elastic
    private static final Field2d m_field = new Field2d();

    // April tag variables
    private static boolean useMegaTag2 = true; // set to false to use MegaTag1. Should test to see which one works better, 1 or 2? Or if they can be combined/we switch between them based on some conditions
    private static boolean doRejectUpdate = false;
    private static String limelightUsed;
    private static LimelightHelpers.PoseEstimate LLPoseEstimate;
    //Get average tag areas (percentage of image), Choose the limelight with the highest average tag area
    private static double limelightFrontAvgTagArea = 0;
    private static double limelightLeftAvgTagArea = 0;
    public static Map<Integer, Pose2d> tagPoseAndymarkMap = new HashMap<>();


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Swerve request to apply during field-centric PIDpath following */
    SwerveRequest.FieldCentric pathPIDRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.FieldCentric pathPIDRequest2 = new SwerveRequest.FieldCentric();

    ProfiledPIDController pathPIDXController = new ProfiledPIDController(SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD, 
                                                                                    new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel));
    ProfiledPIDController pathPIDYController = new ProfiledPIDController(SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD,
                                                                                    new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel));
    ProfiledPIDController pathPIDRotationController = new ProfiledPIDController(SwerveConstants.alignKP, SwerveConstants.alignKI, SwerveConstants.alignKD, 
                                                                                    new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));
    private final Debouncer atGoalDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private int flip_for_red = 1;
    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
        initializeAndyMarkMap();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
        initializeAndyMarkMap();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
        initializeAndyMarkMap();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

        /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    

    //___________________________________________________ Custom Code ___________________________________________________


    private void configureDrivebase() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(1.5, 0, 0),//10,0,0 pre april
                    // PID constants for rotation
                    new PIDConstants(1, 0, 0) //7,0,0 pre april
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        // Configure PID controllers
        pathPIDXController.setTolerance(0.03);
        pathPIDYController.setTolerance(0.05);
        pathPIDRotationController.setTolerance(Math.toRadians(1.5));
        pathPIDRotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Vision setup
        // // Configure AprilTag detection
        // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        //     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{6, 7, 8, 9, 10, 11}); // Only track these tag IDs
        // }
        // else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        // }
        // else{
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        // }
        // LimelightHelpers.SetFiducialDownscalingOverride("limelight-front", 2.0f); // Process at half resolution for improved framerate and reduced range

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            flip_for_red = -1;
        }
    }

    // Move to constants or another java file
    private Double[] Pose2dToDoubleArray(Pose2d pose){
        return new Double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

   


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }


    // ___________________________________________________ Vision Code ___________________________________________________
    


    /**
     * Uses the autobuilder and PathPlanner's navigation grid to pathfind to a pose in real time
     * 
     * @param pose Pose to pathfind to
     * @param endVelocity Velocity at target pose
     */
    public Command pathPlanTo(Pose2d pose, LinearVelocity endVelocity){
        return AutoBuilder.pathfindToPose(pose, SwerveConstants.oTF_Constraints, endVelocity);
    }


    /**
     * Updates the currently used limelight based on which limelight has the largest average tag area.
     */
    private static void chooseLL(boolean useMegaTag2){
        limelightFrontAvgTagArea = NetworkTableInstance.getDefault().getTable(VisionConstants.LL_RIGHT).getEntry("botpose").getDoubleArray(new double[11])[10];
        limelightLeftAvgTagArea = NetworkTableInstance.getDefault().getTable(VisionConstants.LL_LEFT).getEntry("botpose").getDoubleArray(new double[11])[10];
        SmartDashboard.putNumber("Center Limelight Tag Area", limelightFrontAvgTagArea);
        SmartDashboard.putNumber("Left Limelight Tag Area", limelightLeftAvgTagArea);   

        double translationSTD = 15; // safe value ---- ???? got it from them, but seems a bit large. jgt
        if(limelightFrontAvgTagArea > limelightLeftAvgTagArea){
            limelightUsed = "limelight-front";
            translationSTD = VisionConstants.getVisionStd(limelightFrontAvgTagArea);
        }
        else{
            limelightUsed = "limelight-back";
            translationSTD = VisionConstants.getVisionStd(limelightLeftAvgTagArea);
                
        }
        
        // if (useMegaTag2){
        //     visionStandardDeviation = VecBuilder.fill(translationSTD, translationSTD, 9999999); // Don't trust yaw, rely on Pigeon
        // } 
        // else{
        //     visionStandardDeviation = VecBuilder.fill(translationSTD, translationSTD, 999999999); // Use vision yaw reading --- the third value was originally 3 but i changed that back.
        // }

        // SmartDashboard.putString("Limelight Used", limelightUsed);
    }

    /**
     * @return {@code true} if an april tag is in sight, {@code false} otherwise
     */
    public boolean LLHasTag(){
        return getTag() != -1;
    }

    /**
     * Returns the ID of the AprilTag currently visible by the Limelight camera.
     * 
     * @return ID. Returns -1 if no tag is detected.
     */
    public int getTag() {
        int a = (int) NetworkTableInstance.getDefault().getTable(VisionConstants.LL_RIGHT).getEntry("tid").getInteger(-1);
        int b = (int) NetworkTableInstance.getDefault().getTable(VisionConstants.LL_LEFT).getEntry("tid").getInteger(-1);
        return Math.max(a, b);
    }

    /**
     * Creates a command that moves the robot to the position of the AprilTag 
     * detected by the Limelight camera, adjusted by the left branch transformation.
     * 
     * @return A {@link Command} that moves the robot to the transformed position of the detected tag. 
     *         If no tag is visible, a command is returned that logs the absence of a tag.
     */
    private Command pathPIDToTagLeft(int ID){
        SmartDashboard.putNumber("Tag ID used", ID);
        SmartDashboard.putString("Path PID to", tagPoseAndymarkMap.get(ID).transformBy(VisionConstants.rightBranch).toString());

        if (ID != -1)
            return this.pathPIDTo(tagPoseAndymarkMap.get(ID).transformBy(VisionConstants.leftBranch), tagPoseAndymarkMap.get(ID));
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));
    }

    private Command pathPIDToTagMiddle(int ID){
        SmartDashboard.putNumber("Tag ID used", ID);

        if (ID !=-1)
            return this.pathPIDTo(tagPoseAndymarkMap.get(ID).transformBy(VisionConstants.reefAlgae), tagPoseAndymarkMap.get(ID));
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));
        
    }

    /**
     * Creates a command that moves the robot to the position of the AprilTag 
     * detected by the Limelight camera, adjusted by the right branch transformation.
     * 
     * @return A {@link Command} that moves the robot to the transformed position of the detected tag. 
     *         If no tag is visible, a command is returned that logs the absence of a tag.
     */
    private Command pathPIDToTagRight(int ID){
        SmartDashboard.putNumber("Tag ID used", ID);
        SmartDashboard.putString("Path PID to", tagPoseAndymarkMap.get(ID).transformBy(VisionConstants.rightBranch).toString());

        if (ID != -1)
            return this.pathPIDTo(tagPoseAndymarkMap.get(ID).transformBy(VisionConstants.rightBranch), tagPoseAndymarkMap.get(ID));
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));
    }

    public Command pathPIDToTagMiddleSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagMiddle(17)),
                Map.entry(18, this.pathPIDToTagMiddle(18)),
                Map.entry(19, this.pathPIDToTagMiddle(19)),
                Map.entry(20, this.pathPIDToTagMiddle(20)),
                Map.entry(21, this.pathPIDToTagMiddle(21)),
                Map.entry(22, this.pathPIDToTagMiddle(22)),
                Map.entry(6, this.pathPIDToTagMiddle(6)),
                Map.entry(7, this.pathPIDToTagMiddle(7)),
                Map.entry(8, this.pathPIDToTagMiddle(8)),
                Map.entry(9, this.pathPIDToTagMiddle(9)),
                Map.entry(10, this.pathPIDToTagMiddle(10)),
                Map.entry(11, this.pathPIDToTagMiddle(11)))
        , this::getTag);
    }

    public Command pathPIDToTagRightSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagRight(17)),
                Map.entry(18, this.pathPIDToTagRight(18)),
                Map.entry(19, this.pathPIDToTagRight(19)),
                Map.entry(20, this.pathPIDToTagRight(20)),
                Map.entry(21, this.pathPIDToTagRight(21)),
                Map.entry(22, this.pathPIDToTagRight(22)), 
                Map.entry(6, this.pathPIDToTagRight(6)),
                Map.entry(7, this.pathPIDToTagRight(7)),
                Map.entry(8, this.pathPIDToTagRight(8)),
                Map.entry(9, this.pathPIDToTagRight(9)),
                Map.entry(10, this.pathPIDToTagRight(10)),
                Map.entry(11, this.pathPIDToTagRight(11)))
        , this::getTag);
    }

    public Command pathPIDToTagLeftSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagLeft(17)),
                Map.entry(18, this.pathPIDToTagLeft(18)),
                Map.entry(19, this.pathPIDToTagLeft(19)),
                Map.entry(20, this.pathPIDToTagLeft(20)),
                Map.entry(21, this.pathPIDToTagLeft(21)),
                Map.entry(22, this.pathPIDToTagLeft(22)),
                Map.entry(6, this.pathPIDToTagLeft(6)),
                Map.entry(7, this.pathPIDToTagLeft(7)),
                Map.entry(8, this.pathPIDToTagLeft(8)),
                Map.entry(9, this.pathPIDToTagLeft(9)),
                Map.entry(10, this.pathPIDToTagLeft(10)),
                Map.entry(11, this.pathPIDToTagLeft(11)))
        , this::getTag);
    }
    

    /**
     * Creates a command that moves the robot to the specified {@link Pose2d} using PID controllers 
     * for X, Y, and rotation. The command runs until all PID controllers reach their goals, 
     * as determined by the debouncer.
     * 
     * @param goalPose The target {@link Pose2d} the robot should move to.
     * @return A {@link Command} that moves the robot to the specified pose.
     */
    public Command pathPIDTo(Pose2d goalPose, Pose2d tagPose){
        return this.startRun(()->{
            Pose2d currentFieldPose2d = this.getState().Pose;
            Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);
            Pose2d goalTagPose2d = goalPose.relativeTo(tagPose);

            pathPIDXController.reset(currentTagPose2d.getX());
            pathPIDYController.reset(currentTagPose2d.getY());
            pathPIDRotationController.reset(currentTagPose2d.getRotation().getRadians());

            pathPIDXController.setGoal(goalTagPose2d.getX());
            pathPIDYController.setGoal(goalTagPose2d.getY());
            pathPIDRotationController.setGoal(goalTagPose2d.getRotation().getRadians());
        
            }, () -> {
                Pose2d currentFieldPose2d = this.getState().Pose;
                Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);

                Translation2d fieldVelocity = new Translation2d(pathPIDXController.calculate(currentTagPose2d.getX()), pathPIDYController.calculate(currentTagPose2d.getY())).rotateBy(tagPose.getRotation());

                pathPIDRequest
                    .withVelocityX(fieldVelocity.getX() * flip_for_red * 1.5)
                    .withVelocityY(fieldVelocity.getY() * flip_for_red * 1.5)
                    .withRotationalRate(pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians()))
                    .withDeadband(0.05)
                    .withRotationalDeadband(Math.toRadians(2));

                // pathPIDRequest2.withSpeeds(new ChassisSpeeds(fieldVelocity.getX() * flip_for_red, fieldVelocity.getY() * flip_for_red, pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians())));

                this.setControl(pathPIDRequest);

                // SmartDashboard.putNumber("X PID Position Error", pathPIDXController.getPositionError());
                // SmartDashboard.putNumber("X PID Velocity Error", pathPIDXController.getVelocityError());
                // SmartDashboard.putNumber("X PID Velocity setpoint", pathPIDXController.getSetpoint().velocity);
                // SmartDashboard.putNumber("X PID Output", pathPIDXController.calculate(currentTagPose2d.getX()));

                // SmartDashboard.putNumber("Y PID Position Error", pathPIDYController.getPositionError());
                // SmartDashboard.putNumber("Y PID Velocity Error", pathPIDYController.getVelocityError());
                // SmartDashboard.putNumber("Y PID Velocity setpoint", pathPIDYController.getSetpoint().velocity);
                // SmartDashboard.putNumber("Y PID Output", pathPIDYController.calculate(currentTagPose2d.getY()));

                // SmartDashboard.putNumber("Roation PID Position", currentTagPose2d.getRotation().getRadians());
                // SmartDashboard.putNumber("Rotation PID Position Error", pathPIDRotationController.getPositionError());
                // SmartDashboard.putNumber("Rotation PID Velocity Error", pathPIDRotationController.getVelocityError());
                // SmartDashboard.putNumber("Roation PID Velocity Setpoint", pathPIDRotationController.getSetpoint().velocity);
                // SmartDashboard.putNumber("Roation PID Position Setpoint", pathPIDRotationController.getSetpoint().position);
                // SmartDashboard.putNumber("Rotation PID Output", pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians()));

                // SmartDashboard.putBoolean("X PID At Goal", pathPIDXController.atGoal());
                // SmartDashboard.putBoolean("Y PID At Goal", pathPIDYController.atGoal());
                // SmartDashboard.putBoolean("Rotation PID At Goal", pathPIDRotationController.atGoal());

                // SmartDashboard.putNumber("X Total Output", pathPIDXController.calculate(currentTagPose2d.getX()) + pathPIDXController.getSetpoint().velocity);
                // SmartDashboard.putNumber("Y Total Output", pathPIDYController.calculate(currentTagPose2d.getY())+ pathPIDYController.getSetpoint().velocity);
                // SmartDashboard.putNumber("Rotation Total Output", pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians()) + pathPIDRotationController.getSetpoint().velocity);

            }).until(() -> pathPIDAtGoal()).withName("PathPIDTo");
    }

    /**
     * Checks if the PID path follower has been at the goal for the specified debouncer time.
     * 
     * @return {@code true} if the PID controllers for X, Y, and rotation have all been at the goal 
     *         for at least {@code 0.5} seconds (the debouncer time), otherwise {@code false}.
     */
    public boolean pathPIDAtGoal (){
        return atGoalDebouncer.calculate(pathPIDXController.atGoal() && pathPIDYController.atGoal() && pathPIDRotationController.atGoal());
    }

    public void useMegaTag2(boolean input){
        useMegaTag2 = input;
    }

    public void initializeAndyMarkMap(){

        tagPoseAndymarkMap.put(6, new Pose2d(13.474, 3.301, new Rotation2d(Math.toRadians(-60))));
        tagPoseAndymarkMap.put(7, new Pose2d(13.890, 4.021, new Rotation2d(Math.toRadians(0))));
        tagPoseAndymarkMap.put(8, new Pose2d(13.474, 4.740, new Rotation2d(Math.toRadians(60))));
        tagPoseAndymarkMap.put(9, new Pose2d(12.643, 4.740, new Rotation2d(Math.toRadians(120))));
        tagPoseAndymarkMap.put(10, new Pose2d(12.227, 4.021, new Rotation2d(Math.toRadians(180))));
        tagPoseAndymarkMap.put(11, new Pose2d(12.643, 3.301, new Rotation2d(Math.toRadians(-120))));
    
        tagPoseAndymarkMap.put(17, new Pose2d(4.074, 3.301, new Rotation2d(Math.toRadians(-120))));
        tagPoseAndymarkMap.put(18, new Pose2d(3.658, 4.021, new Rotation2d(Math.toRadians(180))));
        tagPoseAndymarkMap.put(19, new Pose2d(4.074, 4.740, new Rotation2d(Math.toRadians(120))));
        tagPoseAndymarkMap.put(20, new Pose2d(4.905, 4.740, new Rotation2d(Math.toRadians(60))));
        tagPoseAndymarkMap.put(21, new Pose2d(5.321, 4.021, new Rotation2d(Math.toRadians(0))));
        tagPoseAndymarkMap.put(22, new Pose2d(4.905, 3.301, new Rotation2d(Math.toRadians(-60))));

        tagPoseAndymarkMap.put(1, new Pose2d(16.5, 0.87, new Rotation2d(Math.toRadians(-45)))); //red left HP //used to be 16.6, 1.11
        tagPoseAndymarkMap.put(2, new Pose2d(16.58, 7.07, new Rotation2d(Math.toRadians(51.9)))); //red right HP
        tagPoseAndymarkMap.put(12, new Pose2d(0.93,0.94, new Rotation2d(Math.toRadians(-125.5)))); //blue right HP //1.06,0.98,-128.3
        tagPoseAndymarkMap.put(13, new Pose2d(1.14, 7.22, new Rotation2d(Math.toRadians(127)))); //blue left HP //1.13, 7.2,125 at calib state, 

        //teleop id 13 1.36,7.17,125
        //teleop id 12, 1.21,1.02,-127.47

    }    
}