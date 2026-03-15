//everything will work. Josh worked his magic.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.Instant;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.CornerShot;
import frc.robot.commands.shoot;
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
        NamedCommands.registerCommand("Run Intake", new InstantCommand(() -> intake.runIntakeLift(0.18)).alongWith(new InstantCommand(() -> intake.runIntake(-0.8))));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand( () -> intake.runIntake(0.0)).alongWith(new InstantCommand(() -> intake.runIntakeLift(-0.12))));
        //NamedCommands.registerCommand("ZeroGyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.05));
        NamedCommands.registerCommand("Climber Up", new InstantCommand(() -> climber.climberUp()));
        NamedCommands.registerCommand("Climber Down", new InstantCommand(() -> climber.climberDown()));

        NamedCommands.registerCommand("Shoot Mode 1", new InstantCommand(() -> Turret.shootMode = 1));

        NamedCommands.registerCommand("Hood Low", new RepeatCommand(new InstantCommand(() -> shooter.hoodLow())).finallyDo(interrupted -> { 
            shooter.automaticHood();
        }));

        NamedCommands.registerCommand("Corner Shot", new CornerShot(shooter, intake));




    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        chooser.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        chooser.addOption("Left Auto Path Only", new PathPlannerAuto("Left Auto Path Only"));

        return chooser;
    }

    


}
