// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.shoot;
import frc.robot.commands.shootmath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.commands.ShootOnTheMove;


public class RobotContainer {

    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
    ShootingMath shooterAimer = new ShootingMath(new Transform3d());

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

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
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

        //testing.y().whileTrue(drivetrain.applyRequest(() -> brake));
        testing.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-testing.getLeftY(), -testing.getLeftX()))
        ));

        // add import
        //operator.x().

        driver.L2().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        driver.L2().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        driver.R2().whileTrue(new shoot(shooter, intake));
        driver.R2().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopIntake())));
        
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
        driver.square().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        // in configureBindings():
        //driver.a().whileTrue(new ShootOnTheMove(drivetrain, shooter, turret, intake));
        //driver.a().whileFalse(new InstantCommand(() -> shooter.stopShooter()));
        //driver.a().whileTrue(new InstantCommand(() -> shooter.setShooterRPM(ShootingMath.initialCalcShot)).alongWith(new InstantCommand(hood.setHoodAngleDegrees()))
        testing.a().whileTrue(new shootmath(shooter, intake, drivetrain));
        testing.a().whileFalse(new InstantCommand(() -> shooter.stopShooter()).alongWith(new InstantCommand(() -> intake.hopper(0, 0))));

        //TODO: TOGGLE TO MANUAL 

        //MANUAL TURRET
        testing.x().whileTrue(new InstantCommand(() -> turret.manualTurret(0.1)));
        testing.x().whileFalse(new InstantCommand(() -> turret.stopTurret()));

        //testing.y().whileTrue(new InstantCommand(() -> turret.manualTurret(-0.1)));
        //testing.y().whileFalse(new InstantCommand(() -> turret.stopTurret()));
        

        //RUN MACROINTAKE
        testing.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        testing.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));

        operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        operator.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));
        
        operator.povUp().whileTrue(new InstantCommand(() -> intake.manualIntake(-0.5)));
        operator.povUp().whileFalse(new InstantCommand(() -> intake.zeroIntake()).alongWith(new InstantCommand(() -> intake.manualIntake(0))));

        operator.x().whileTrue(new InstantCommand(() -> shooter.setHood(-0.64)).alongWith(new InstantCommand(() -> shooter.shootSpeed(65.0))));
        operator.x().whileFalse(new InstantCommand(() -> shooter.setHood(0)).alongWith(new InstantCommand(() -> shooter.stopShooter())));
        
        operator.b().whileTrue(new InstantCommand(() -> Turret.shootMode = 2).andThen(new shoot(shooter, intake)));
        operator.b().whileFalse(new InstantCommand(() -> Turret.shootMode = 4).alongWith(new InstantCommand(() -> shooter.stopShooter())));

        operator.start().whileTrue(new InstantCommand(() -> intake.hopper(-0.6, 0.85)).alongWith(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0.8)))));
        operator.start().whileFalse(new InstantCommand(() -> intake.hopper(0, 0)).alongWith(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.stopIntake()))));

        operator.rightStick().whileTrue(new InstantCommand(() -> intake.zeroIntake()));
        operator.leftStick().whileTrue(new InstantCommand(() -> shooter.zeroHood()));


        //operator.rightBumper().whileTrue(new InstantCommand (() -> MAIL OUPOST)) TODO: FILL IN MAIL OUTPOST COMMAND
        //operator.rightBumper().whileFalse(new InstantCommand (() -> shooter.stopShooter()));

        //operator.leftBumper().whileTrue(new InstantCommand (() -> MAIL DEPOT)) TODO: FILL IN MAIL DEPOT COMMAND
        //operator.leftBumper().whileFalse(new InstantCommand (() -> shooter.stopShooter()));

        
        //SHOOT COMMAND
        testing.rightTrigger().whileTrue(new shoot(shooter, intake));
        testing.rightTrigger().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopIntake())));
    
        operator.rightTrigger().whileTrue(new shoot(shooter, intake));
        operator.rightTrigger().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopIntake())));

        //TURRET SHOT MODE CHANGE  //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.
        testing.rightStick().onTrue(new InstantCommand(() -> turret.shootModeChange(true)));
        testing.leftStick().onTrue(new InstantCommand(() -> turret.shootModeChange(false)));

        //MANUAL SHOOTER HOOD
        testing.povLeft().onTrue(new InstantCommand(() -> shooter.changeHoodDown()));
        testing.povRight().onTrue(new InstantCommand(() -> shooter.changeHoodUp()));

        testing.y().whileTrue(new InstantCommand(() -> shooter.runHood(0.5)));
        testing.b().whileTrue(new InstantCommand(() -> shooter.runHood(-0.5)));
        testing.b().whileFalse(new InstantCommand(() -> shooter.stopHood()));
        testing.y().whileFalse(new InstantCommand(() -> shooter.stopHood()));


        //MANUAL SHOOTER POWER
        testing.povUp().onTrue(new InstantCommand(() -> shooter.changeShooterUp()));
        // testing.povUp().whileTrue(new InstantCommand(() -> shooter.setHood(0.1)));
        // testing.povUp().whileFalse(new InstantCommand(() -> shooter.stopHood()));
        testing.povDown().onTrue(new InstantCommand(() -> shooter.changeShooterDown()));

        //RUN SHOOTER POWER
        // driver.rightBumper().whileTrue(new InstantCommand ( () -> shooter.manualShooterSPEED()));
        // driver.rightBumper().whileFalse(new InstantCommand ( () -> shooter.stopShooter()));

        testing.rightBumper().onTrue(new InstantCommand(() -> shooter.zeroHood()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        testing.back().and(testing.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testing.back().and(testing.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        testing.start().and(testing.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testing.start().and(testing.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        testing.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         drivetrain.applyRequest(() -> idle)
    //     );
    // }
    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
