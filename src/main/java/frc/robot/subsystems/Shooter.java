// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(shooterConstants.shooterLeft);
  private TalonFX shooterRight = new TalonFX(shooterConstants.shooterRight);
  private TalonFX hood = new TalonFX(shooterConstants.hood);

  double shooterPower, hoodPosition;


  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Intake. */
  public Shooter() {

    var talonFXConfigs = new TalonFXConfiguration();

    // talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hood.getConfigurator().apply(config);

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2.5; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    config.Slot0 = slot0Configs;

    m_motmag.EnableFOC = true;


    hood.getConfigurator().apply(motionMagicConfigs);
    hood.getConfigurator().apply(slot0Configs); 
    //hood.getConfigurator().apply(config);
  }

  public void shoot(/*double speed*/) {
    shooterLeft.set(shooterPower); //was speed
    shooterRight.set(shooterPower);
  }
  public void stopShooter() {
    shooterLeft.set(0); 
    shooterRight.set(0);
  }
  public void hood (double speed) {
    hood.set(speed);
  }

  public void setHood(/*double position*/) {
    hood.setControl(m_motmag.withPosition(hoodPosition)); //was position
  }

  public void changeShooterUp() {
    shooterPower += 0.05;
  }
  public void changeShooterDown() {
    shooterPower -= 0.05;
  }

  public void changeHoodUp() {
    hoodPosition += 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));

  }
  public void changeHoodDown() {
    hoodPosition -= 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));
  }

  @Override
  public void periodic() {
    System.out.println("hood position " + hoodPosition);
    System.out.println("shooter power" + shooterPower);
    // This method will be called once per scheduler run
  }
}

