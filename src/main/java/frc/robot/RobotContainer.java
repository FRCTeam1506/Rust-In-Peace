// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.PresetShots;
import frc.robot.commands.shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


public class RobotContainer {

    
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController testing = new CommandXboxController(2);
    public final CommandPS4Controller driver = new CommandPS4Controller(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    //INITIALIZE SUBSYTEMS
    Turret turret = new Turret(drivetrain);
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Autos autos = new Autos(drivetrain, intake, shooter, turret);
    private boolean m_isAutoMode = false; 


    private final SendableChooser<Command> autoChooser;
    private SendableChooser<Command> autoChooserManual;
    public static SendableChooser<String> lazyAuto2000 = new SendableChooser<String>();
    public static SendableChooser<Integer> autoDriveLocation = new SendableChooser<Integer>();


    public RobotContainer() {
        autos.makeNamedCommands();
    
        autoChooserManual = new SendableChooser<Command>();
        autoChooserManual = autos.configureChooser(autoChooserManual);

        autoChooser = AutoBuilder.buildAutoChooser("Left Auto Path Only");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("lazyAuto2000", lazyAuto2000);

        SmartDashboard.putData("SmartPathfinding", autoDriveLocation); 

        SmartDashboard.putData("Auto Mode 2000", autoChooserManual);

        //shooter.setDefaultCommand(Commands.run(() -> shooter.stopShooter(), shooter));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        //For testing
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-testing.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-testing.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-testing.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        //For driver
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //TESTING CONTROLLER
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-testing.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-testing.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-testing.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            //DRIVER CONTROLLER
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        testing.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-testing.getLeftY(), -testing.getLeftX()))
        ));


        //DRIVER CONTROLS MANUAL VERSION (COMP): :(
        driver.L2().whileTrue(new InstantCommand(() -> intake.runIntakeLift(0.18)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        driver.L2().whileFalse(new InstantCommand(() -> intake.runIntakeLift(-0.12)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        driver.R2().whileTrue(new shoot(shooter, intake));
        driver.R2().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));
        
        //DRIVER CONTROLS AUTO VERSION:

        //Shoot mode
        // driver.L3().onTrue(new InstantCommand(() -> turret.shootModeChange(true)));
        // driver.R3().onTrue(new InstantCommand(() -> turret.shootModeChange(false)));

        //driver.L2().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        //driver.L2().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        //TO TEST: for default hood low
        // driver.share().onTrue(Commands.runOnce(() -> {
        //     m_isAutoMode = !m_isAutoMode;
        //     SmartDashboard.putBoolean("Shooting Auto Mode", m_isAutoMode);
        // }));

        // driver.R2().whileTrue(
        //     Commands.either(
        //         new shoot(shooter, intake), 
        //         Commands.run(() -> {        
        //             shooter.incrementalShooter();
        //             intake.runIntake(-0.8);
        //         }, shooter, intake),
        //         () -> m_isAutoMode 
        //     )
        // );

        // driver.R2().onFalse(new InstantCommand(() -> {
        //     shooter.stopShooter();
        //     intake.stopAllIntake();
        // }));


        //Shoot
        //driver.R2().whileTrue(new shoot(shooter, intake));
        //driver.R2().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));
        
        driver.circle().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        
        //PRESET SHOTS:
        //Trench shot preset
        driver.square().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.trenchShotRPS, Constants.presetShots.trenchShotHoodAngle));
        driver.square().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));
        
        //Corner shot preset
        driver.triangle().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.backCornerShotRPS, Constants.presetShots.backCornerShotHoodAngle));
        driver.triangle().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));

        //Close shot preset
        driver.cross().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.closeShotRPS, Constants.presetShots.closeShotHoodAngle));
        driver.cross().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));

        //Rev intake to help feed balls
        driver.R1().whileTrue(new InstantCommand(() -> intake.runIntake(0.8)));
        driver.R1().whileFalse(new InstantCommand(() -> intake.runIntake(0)));

        driver.povUp().whileTrue(new InstantCommand(() -> climber.runClimber(-0.5)));
        driver.povUp().whileFalse(new InstantCommand(() -> climber.runClimber(0)));
        driver.povDown().whileTrue(new InstantCommand(() -> climber.runClimber(0.5)));
        driver.povDown().whileFalse(new InstantCommand(() -> climber.runClimber(0)));

        // driver.povLeft().whileTrue(new InstantCommand(() -> turret.manualTurret(0.25)));
        // driver.povRight().whileTrue(new InstantCommand(() -> turret.manualTurret(-0.25)));
        // driver.povLeft().whileFalse(new InstantCommand(() -> turret.manualTurret(0)));
        // driver.povRight().whileFalse(new InstantCommand(() -> turret.manualTurret(0)));

        
        


        //TO IMPLEMENT:
        //driver.R1().whileTrue(new InstantCommand(() -> MAIL OUTPOST))
        //driver.R1().whileFalse(new InstantCommand(() -> shooter.stopShooter()));

        //driver.L1().whileTrue(new InstantCommand(() -> MAIL DEPOT))
        //driver.L1().whileFalse(new InstantCommand(() -> shooter.stopShooter()));

        //IMPLEMENT CLIMBER CONTROL WHEN CLIMBER IS READY
        // driver.povUp().whileTrue(new InstantCommand(() -> climber.climberUp()));
        // driver.povDown().whileFalse(new InstantCommand(() -> climber.stop()));
        // driver.povDown().whileTrue(new InstantCommand(() -> climber.climberDown()));
        // driver.povUp().whileFalse(new InstantCommand(() -> climber.stop()));

        //RUN SHOOTER POWER
        // driver.rightBumper().whileTrue(new InstantCommand ( () -> shooter.manualShooterSPEED()));
        // driver.rightBumper().whileFalse(new InstantCommand ( () -> shooter.stopShooter()));

        //TODO: TOGGLE TO MANUAL 

        //OPERATOR CONTROLS MANUAL (COMP):
        operator.start().whileTrue(new InstantCommand(() -> intake.hopper(-0.6, 0.85)).alongWith(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0.8)))));
        operator.start().whileFalse(new InstantCommand(() -> intake.hopper(0, 0)).alongWith(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.stopAllIntake()))));


        //OPERATOR CONTROLS:
        //operator.b().onTrue(new InstantCommand(() -> turret.shootModeChange(true)));
        //operator.b().onTrue(new InstantCommand(() -> turret.shootModeChange(false)));     
        //Run intake lift up, then zero when dpad up is released.
        //operator.povUp().whileTrue(new InstantCommand(() -> intake.runIntakeLift(-0.5)));
        //operator.povUp().onFalse(new InstantCommand(() -> intake.zeroIntakeLift()).alongWith(new InstantCommand(() -> intake.runIntakeLift(0))));
        // operator.povUp().whileTrue(new InstantCommand(() -> turret.shootModeChange(true)).andThen(new InstantCommand(() -> turret.manualTurret(0.25))));
        // operator.povUp().whileFalse(new InstantCommand(() -> turret.manualTurret(0)));
        // operator.povUp().whileTrue(new InstantCommand(() -> turret.shootModeChange(true)).andThen(new InstantCommand(() -> turret.manualTurret(0.25))));
        // operator.povRight().whileFalse(new InstantCommand(() -> turret.manualTurret(0)));

        //PRESET SHOTS:
        //Trench shot preset
        operator.x().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.trenchShotRPS, Constants.presetShots.trenchShotHoodAngle));
        operator.x().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));
        
        //Corner shot preset
        operator.y().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.backCornerShotRPS, Constants.presetShots.backCornerShotHoodAngle));
        operator.y().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));

        //Close shot preset
        operator.a().whileTrue(new PresetShots(shooter, intake, Constants.presetShots.closeShotRPS, Constants.presetShots.closeShotHoodAngle));
        operator.a().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));

        //Outtake - runs intake in reverse and lowers intake lift to outtake balls
        operator.start().whileTrue(new InstantCommand(() -> intake.hopper(-0.6, 0.85)).alongWith(new InstantCommand(() -> intake.runIntakeLift(0.2)).alongWith(new InstantCommand(() -> intake.runIntake(0.8)))));
        operator.start().whileFalse(new InstantCommand(() -> intake.hopper(0, 0)).alongWith(new InstantCommand(() -> intake.runIntakeLift(-0.2)).alongWith(new InstantCommand(() -> intake.stopAllIntake()))));

        //Macrointake
        //BY POSITION:
        //operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        //operator.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        //BY POWER:
        operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.runIntakeLift(0.2)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        operator.leftTrigger().whileFalse(new InstantCommand(() -> intake.runIntakeLift(-0.2)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        //Shooter
        operator.rightTrigger().whileTrue(new shoot(shooter, intake));
        operator.rightTrigger().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));

        //Rev intake to help feed balls
        //operator.rightBumper().whileTrue(new InstantCommand(() -> intake.runIntake(0.8)));
        //operator.rightBumper().whileFalse(new InstantCommand(() -> intake.runIntake(0)));
        //operator.rightBumper().whileTrue(new InstantCommand (() -> MAIL OUPOST)) TODO: FILL IN MAIL OUTPOST COMMAND

        //operator.rightStick().onTrue(new InstantCommand(() -> shooter.toggleManualHood = true));
        //operator.leftStick().onTrue(new InstantCommand(() -> shooter.toggleManualHood = false));
        operator.rightBumper().whileTrue(new InstantCommand(() -> shooter.changeHoodUp()));
        operator.leftBumper().whileTrue(new InstantCommand(() -> shooter.changeHoodDown()));
        operator.rightBumper().whileFalse(new InstantCommand(() -> shooter.stopHood()));
        operator.leftBumper().whileFalse(new InstantCommand(() -> shooter.stopHood()));

        //operator.rightBumper().whileFalse(new InstantCommand (() -> shooter.stopShooter()));

        //operator.leftBumper().whileTrue(new InstantCommand (() -> MAIL DEPOT)) TODO: FILL IN MAIL DEPOT COMMAND
        //operator.leftBumper().whileFalse(new InstantCommand (() -> shooter.stopShooter()));

        //Manual Turret


        //TESTING CONTROLS:
        //Manual Intake Lift:
        testing.a().whileTrue(new InstantCommand(() -> intake.runIntakeLift(0.2)));
        testing.a().whileFalse(new InstantCommand(() -> intake.runIntakeLift(0)));
        testing.y().whileTrue(new InstantCommand(() -> intake.runIntakeLift(-0.2)));
        testing.y().whileTrue(new InstantCommand(() -> intake.runIntakeLift(0)));



        //MANUAL TURRET
        //testing.x().whileTrue(new InstantCommand(() -> turret.manualTurret(0.1)));
        testing.x().whileFalse(new InstantCommand(() -> turret.stopTurret()));

        //testing.y().whileTrue(new InstantCommand(() -> turret.manualTurret(-0.1)));
        //testing.y().whileFalse(new InstantCommand(() -> turret.stopTurret()));
        // testing.y().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.hoodLow())).finallyDo(interrupted -> { 
        //     shooter.automaticHood();
        // }));
        
        //RUN MACROINTAKE
        testing.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        testing.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));

        //SHOOT COMMAND
        testing.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        testing.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        testing.rightTrigger().whileTrue(new shoot(shooter, intake));
        testing.rightTrigger().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopAllIntake())));
    
        //TURRET SHOT MODE CHANGE  //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.
        testing.rightStick().onTrue(new InstantCommand(() -> turret.shootModeChange(true)));
        testing.leftStick().onTrue(new InstantCommand(() -> turret.shootModeChange(false)));

        //testing.y().whileTrue(new InstantCommand(() -> shooter.runHood(0.7))); //Run hood up by setting speed
        testing.b().whileTrue(new InstantCommand(() -> shooter.runHood(-0.7))); //Run hood down by setting speed
        testing.b().whileFalse(new InstantCommand(() -> shooter.stopHood()));
        //testing.y().whileFalse(new InstantCommand(() -> shooter.stopHood()));


        //MANUAL SHOOTER SPEED AND HOOD ANGLE ADJUSTMENT
        testing.povLeft().onTrue(new InstantCommand(() -> shooter.changeShooterUp())); //Increase manual shooter speed
        testing.povRight().onTrue(new InstantCommand(() -> shooter.changeShooterDown())); //Decrease manual shooter speed
        testing.povUp().whileTrue(new InstantCommand(() -> shooter.changeHoodUp())); //Increase hoodPosition - Increase manual hood angle
        testing.povDown().whileTrue(new InstantCommand(() -> shooter.changeHoodDown())); //Decrease hoodPosition - Decrease manual hood angle
        testing.povDown().whileFalse(new InstantCommand(() -> shooter.stopHood())); 
        testing.povUp().whileFalse(new InstantCommand(() -> shooter.stopHood()));

        testing.rightBumper().onTrue(new InstantCommand(() -> shooter.zeroHood())); //Zero hood encoder - untested on new hood

        // Reset the field-centric heading on left bumper press.
        testing.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
