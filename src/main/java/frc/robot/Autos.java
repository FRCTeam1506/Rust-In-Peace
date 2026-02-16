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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Autos {
    
    static Intake intake;
    static Shooter shooter;
    static Turret turret;
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
        NamedCommands.registerCommand("Run Shooter ", new shoot(shooter, intake));
        // NamedCommands.registerCommand("Run Intake ", new ParallelDeadlineGroup(new WaitCommand(1.5), new InstantCommand( () -> intake.runIntake(0.5))));
        NamedCommands.registerCommand("Run Intake ", new InstantCommand( () -> intake.runIntake(0.5)).alongWith(new InstantCommand(() -> intake.intakeLift(intakeConstants.loweredIntake))));
        NamedCommands.registerCommand("Stop Intake ", new InstantCommand( () -> intake.runIntake(0.0)).alongWith(new InstantCommand(() -> intake.intakeLift(intakeConstants.upIntake))));

        //NamedCommands.registerCommand("ZeroGyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.05));


    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        //chooser.addOption("Left", new PathPlannerAuto("Left"));

        return chooser;
    }


}
