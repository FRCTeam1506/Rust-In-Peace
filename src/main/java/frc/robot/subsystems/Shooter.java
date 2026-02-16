// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(shooterConstants.shooterLeft);
  private TalonFX shooterRight = new TalonFX(shooterConstants.shooterRight);
  private TalonFX hood = new TalonFX(shooterConstants.hood);

  double shooterPower, hoodPosition;

  InterpolatingDoubleTreeMap finalHoodPosition = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap finalShooterPower = new InterpolatingDoubleTreeMap();


public class FullShooterParams {

  double rpm;
  double hoodAngle;
  double timeOfFlight;

  public FullShooterParams(double rpm, double hoodAngle, double timeOfFlight) {
    this.rpm = rpm;
    this.hoodAngle = hoodAngle;
    this.timeOfFlight = timeOfFlight;
  }
}

  // private static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<Double, FullShooterParams>();
  // static {
  //       SHOOTER_MAP.put(1.5, new FullShooterParams(2800.0, 35.0, 0.38));
  //       SHOOTER_MAP.put(2.0, new FullShooterParams(3100.0, 38.0, 0.45));
  //       SHOOTER_MAP.put(2.5, new FullShooterParams(3400.0, 42.0, 0.52));
  //       SHOOTER_MAP.put(3.0, new FullShooterParams(3650.0, 46.0, 0.60));
  //       SHOOTER_MAP.put(3.5, new FullShooterParams(3900.0, 50.0, 0.68));
  //       SHOOTER_MAP.put(4.0, new FullShooterParams(4100.0, 54.0, 0.76));
  //       SHOOTER_MAP.put(4.5, new FullShooterParams(4350.0, 58.0, 0.85));
  //       SHOOTER_MAP.put(5.0, new FullShooterParams(4550.0, 62.0, 0.94));

  // }




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

    finalHoodPosition.put(1.524, -0.6);
    finalHoodPosition.put(2.4384, -0.6);
    finalHoodPosition.put(3.048, -1.15);
    finalHoodPosition.put(4.114, -1.3);
    finalHoodPosition.put(4.572, -1.45);

    finalShooterPower.put(1.524, 0.525);
    finalShooterPower.put(2.4384, 0.575);
    finalShooterPower.put(3.048, 0.6);
    finalShooterPower.put(4.114, 0.725);
    finalShooterPower.put(4.572, 0.75);

  }

  public void shoot(double speed) {
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

  public void setHood(double position) {
    hood.setControl(m_motmag.withPosition(hoodPosition)); //was position, then Constants.shooterConstants.hoodPosition
  }

  public void changeShooterUp() {
    shooterPower += 0.025;
  }
  public void changeShooterDown() {
    shooterPower -= 0.025;
  }

  public void changeHoodUp() {
    hoodPosition += 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));

  }
  public void changeHoodDown() {
    hoodPosition -= 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));
  }

  public void mainHoodAngle() {
    hood.setControl(m_motmag.withPosition(finalHoodPosition.get(Constants.distToGoal)));
  }

  public void mainShooterPower() {
    shooterLeft.set(finalShooterPower.get(Constants.distToGoal));
    shooterRight.set(finalShooterPower.get(Constants.distToGoal));
  }

  @Override
  public void periodic() {
    //System.out.println("hood position " + Constants.shooterConstants.hoodPosition);
    //System.out.println("shooter power" + Constants.shooterConstants.shooterPower);
    // This method will be called once per scheduler run

    // shooterConstants.shooterPower  = (Constants.distToGoal * 0.0777909) + 0.390793;
    // shooterConstants.hoodPosition  = (Constants.distToGoal * -0.5302696) - 0.0697538;

    // if(shooterConstants.hoodPosition >= Constants.shooterConstants.hoodMaxPosition) {
    //   shooterConstants.hoodPosition = Constants.shooterConstants.hoodMaxPosition;
    // }
    
    // if(shooterConstants.hoodPosition <= Constants.shooterConstants.hoodMinPosition) {
    //   shooterConstants.hoodPosition = Constants.shooterConstants.hoodMinPosition;
    // }

    //System.out.println("dist to goal" + Constants.distToGoal);
    
    // setHood(shooterConstants.hoodPosition);
    // shoot(shooterConstants.shooterPower);
  }
}

