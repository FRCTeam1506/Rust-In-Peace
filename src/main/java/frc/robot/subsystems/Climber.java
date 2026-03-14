// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climber = new TalonFX(climberConstants.climberID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public Climber() {
    var climberConfigs = new TalonFXConfiguration();
    climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //general configurations
    climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfigs.CurrentLimits.StatorCurrentLimit = 80;

    //motion magic configurations
    climberConfigs.MotionMagic.MotionMagicCruiseVelocity = 180; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    climberConfigs.MotionMagic.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    climberConfigs.MotionMagic.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    //pid configurations
    climberConfigs.Slot0.kS = 0.24; 
    climberConfigs.Slot0.kV = 0.12; 
    climberConfigs.Slot0.kP = 2; //4.8
    climberConfigs.Slot0.kI = 0;
    climberConfigs.Slot0 .kD = 0.1;

    climber.getConfigurator().apply(climberConfigs);
    }

  public void runClimber (double power) {
    climber.set(power);
  } 

  public void climberUp () {
    climber.setControl(m_motmag.withPosition(Constants.climberConstants.climberUpPosition));
  }

  public void climberDown () {
    climber.setControl(m_motmag.withPosition(Constants.climberConstants.climberDownPosition));

  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("Turret Position ", climber.getPosition().getValueAsDouble());


    // This method will be called once per scheduler run
  }
}
