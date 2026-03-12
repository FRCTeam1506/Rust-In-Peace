// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CornerShot extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public CornerShot(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new InstantCommand(() -> shooter.shoot(0.75)), //0.75
      //new InstantCommand(() -> shooter.LUTHood()),
      new InstantCommand(() -> shooter.manualShooter(Constants.presetShots.trenchShotRPM)),
      new InstantCommand(() -> shooter.setHood(Constants.presetShots.trenchShotHoodAngle)),
      new InstantCommand(() -> intake.runIntake(-0.2)),
      new InstantCommand(() -> intake.hopper(0, 0)),     
      new WaitCommand(0.2),
      new InstantCommand(() -> intake.hopper(0, -0.4)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.hopper(0.6,-0.85))
    );
  }
}
