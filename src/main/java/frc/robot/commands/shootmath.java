// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.ShootingMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootmath extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public shootmath(Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new InstantCommand(() -> shooter.shoot(0.75)), //0.75
      new InstantCommand(() -> shooter.setShooterToMathRPMFromMPS(ShootingMath.findIdealVelocityAndAngle(drivetrain.getState().Pose, drivetrain.getState().Speeds).getVelocity().magnitude())),
      new InstantCommand(() -> shooter.setHoodAngleDegrees(Math.toDegrees(ShootingMath.findIdealVelocityAndAngle(drivetrain.getState().Pose, drivetrain.getState().Speeds).getPitchAngle().magnitude()))),
      new WaitCommand(1),
      new InstantCommand(() -> intake.hopper(0.4, 0.0)),
      new WaitCommand(0.25),
      new InstantCommand(() -> intake.hopper(0.6,-0.85))
      // new InstantCommand(() -> shooter.shoot(0.75)) //0.5
      //new InstantCommand(() -> shooter.mainShooterPower())
    );
  }
}
