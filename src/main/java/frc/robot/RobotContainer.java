// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

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

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    //INITIALIZE SUBSYTEMS
    Turret turret = new Turret(drivetrain);
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Autos autos = new Autos(drivetrain, intake, shooter, turret);



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

        driver.y().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));



        //RUN MACROINTAKE
        driver.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.loweredIntake)).alongWith(new InstantCommand(() -> intake.runIntake(-1))));
        driver.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeLift(Constants.intakeConstants.upIntake)).alongWith(new InstantCommand(() -> intake.runIntake(0))));

        //SHOOT COMMAND
        driver.rightTrigger().whileTrue(new shoot(shooter, intake));
        driver.rightTrigger().whileFalse(new InstantCommand( () -> shooter.stopShooter()).alongWith(new InstantCommand( () -> intake.stopIntake())));
    
        //TURRET SHOT MODE CHANGE  //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.
        driver.rightStick().onTrue(new InstantCommand(() -> turret.shootModeChange(true)));
        driver.leftStick().onTrue(new InstantCommand(() -> turret.shootModeChange(false)));

        //MANUAL SHOOTER HOOD
        driver.povLeft().onTrue(new InstantCommand(() -> shooter.changeHoodDown()));
        driver.povRight().onTrue(new InstantCommand(() -> shooter.changeHoodUp()));

        //MANUAL SHOOTER POWER
        driver.povUp().onTrue(new InstantCommand(() -> shooter.changeShooterUp()));
        driver.povDown().onTrue(new InstantCommand(() -> shooter.changeShooterDown()));

        //RUN SHOOTER POWER
        // driver.rightBumper().whileTrue(new InstantCommand ( () -> shooter.manualShooterSPEED()));
        // driver.rightBumper().whileFalse(new InstantCommand ( () -> shooter.stopShooter()));

        driver.rightBumper().onTrue(new InstantCommand(() -> shooter.zeroHood()));

        driver.b().whileTrue(new InstantCommand ( () -> shooter.manualShooterRPM()));
        driver.b().whileFalse(new InstantCommand ( () -> shooter.stopShooter()));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
