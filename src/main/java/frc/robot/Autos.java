//everything will work. Josh worked his magic.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.Instant;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Autos {
    
    static Intake intake;
    static Shooter shooter;
    static Turret turret;
    static Climber climber;
    static CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public Autos(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, Turret turret){
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands(){
        NamedCommands.registerCommand("Run Shooter", new shoot(shooter, intake));
        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> shooter.stopShooter()).alongWith(new InstantCommand(() -> intake.hopper(0, 0))));
        //NamedCommands.registerCommand("Continuous Shooter", new RunCommand(null, null))
        // NamedCommands.registerCommand("Run Intake ", new ParallelDeadlineGroup(new WaitCommand(1.5), new InstantCommand( () -> intake.runIntake(0.5))));
        NamedCommands.registerCommand("Run Intake", new InstantCommand( () -> intake.runIntake(0.5)).alongWith(new InstantCommand(() -> intake.intakeLift(intakeConstants.loweredIntake))));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand( () -> intake.runIntake(0.0)).alongWith(new InstantCommand(() -> intake.intakeLift(intakeConstants.upIntake))));
        //NamedCommands.registerCommand("ZeroGyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.05));
        NamedCommands.registerCommand("Climber Up", new InstantCommand(() -> climber.climberUp()));
        NamedCommands.registerCommand("Climber Down", new InstantCommand(() -> climber.climberDown()));

        NamedCommands.registerCommand("Shoot Mode 4", new InstantCommand(() -> Turret.shootMode = 1));

        NamedCommands.registerCommand("Hood Low", new InstantCommand(() -> shooter.hoodLow()));

        NamedCommands.registerCommand("Corner Shot Hood", new InstantCommand(() -> shooter.setHood(Constants.shooterConstants.cornerShotHoodAngle)));




    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        chooser.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        chooser.addOption("Left Auto Path Only", new PathPlannerAuto("Left Auto Path Only"));

        return chooser;
    }

    


}
