// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(shooterConstants.shooterLeft);
  private TalonFX shooterRight = new TalonFX(shooterConstants.shooterRight);
  private TalonFX hood = new TalonFX(shooterConstants.hood);

  double shooterPower = 50;
  double hoodPosition;

  public static double shooterRPS;
  public static double wheelSurfaceSpeed;
  public static double exitVelocity;
  public static double hoodAngleDegrees;
  public static double hoodAngleRadians;
  public static double vX;
  public static double calculatedTOF;

  final VelocityVoltage speedControl = new VelocityVoltage(12);


  //FOR ALL OF THESE, KEY IS DISTANCE, OUTPUT IS NAME OF THE TABLE
  public InterpolatingDoubleTreeMap finalHoodPosition = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap finalShooterRPS = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();


  double mainShooterRPS, mainHoodAngle;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Intake. */
  public Shooter() {

    var talonFXConfigs = new TalonFXConfiguration();
    var hoodConfigs = new TalonFXConfiguration();

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
    var slot0hoodConfigs = hoodConfigs.Slot0;
    slot0hoodConfigs.kS = 0.24; // add 0.24 V to overcome friction
    slot0hoodConfigs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0hoodConfigs.kP = 10; //2.5
    slot0hoodConfigs.kI = 0;
    slot0hoodConfigs.kD = 0.25;

    slot0hoodConfigs.kV = 0.12; // apply 12 V for a target velocity of 100 rps

    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 100; //2.5
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.25;

    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps

    config.Slot0 = slot0Configs;
    

    m_motmag.EnableFOC = true;


    hood.getConfigurator().apply(motionMagicConfigs);
    hood.getConfigurator().apply(slot0hoodConfigs); 
    shooterLeft.getConfigurator().apply(slot0Configs);
    shooterRight.getConfigurator().apply(slot0Configs);
    //hood.getConfigurator().apply(config);

    finalHoodPosition.put(1.68, 0.0); //somewhat close to the hub
    finalHoodPosition.put(2.31, 0.0); // closer to the climbing rack
    finalHoodPosition.put(2.87, -0.64); //at the climbing rack
    finalHoodPosition.put(3.94, -1.24); //back against the back wall
    finalHoodPosition.put(4.27, -1.1865234375); //in the square thing
    finalHoodPosition.put(5.03, -1.66796875); //in the back corner
    finalHoodPosition.put(3.23, -0.66); //auton starting point




    finalShooterRPS.put(1.68, 49.0);
    finalShooterRPS.put(2.31, 53.0);
    finalShooterRPS.put(2.87, 65.0);
    finalShooterRPS.put(3.94, 76.0);// consider lowering because it is overshooting
    finalShooterRPS.put(4.27, 75.0);
    finalShooterRPS.put(5.03, 83.0);
    //finalShooterRPS.put(3.23, 70);




    timeOfFlight.put(1.68, 1.0);
    timeOfFlight.put(2.31, 1.12);
    timeOfFlight.put(2.87, 1.4);
    timeOfFlight.put(3.94, 1.4);
    timeOfFlight.put(4.27, 1.5);
    timeOfFlight.put(5.03, 1.55);
    //timeOfFlight.put(3.23, 1.55);



  }

  public void shoot(double speed) {
    //shooterLeft.set(shooterPower/100); //was speed
    //shooterRight.set(shooterPower/100);
    shooterLeft.setControl(speedControl.withVelocity(shooterPower));
    shooterRight.setControl(speedControl.withVelocity(shooterPower));
  }

  public void stopShooter() {
    shooterLeft.set(0); 
    shooterRight.set(0);
  }

  public void manualShooterSPEED() {
    shooterLeft.set(shooterPower/100);
    shooterRight.set(shooterPower/100);
  }

  public void shootSpeed(double speed) {
    shooterLeft.setControl(speedControl.withVelocity(speed));
    shooterRight.setControl(speedControl.withVelocity(speed));
  }
  public void zeroHood() {
    hood.setPosition(0);
  }

  public void setDifferentShooterRPMs(double bottomRPM, double topRPM) {
    shooterLeft.setControl(speedControl.withVelocity(bottomRPM/60)); //MAKE SURE THAT BOTTOM IS "LEFT" AND TOP IS "RIGHT" 
    shooterRight.setControl(speedControl.withVelocity(topRPM/60));
  }

  public void hood (double speed) {
    hood.set(speed);
  }

  public void setHood(double position) {
    hood.setControl(m_motmag.withPosition(hoodPosition)); //was position, then Constants.shooterConstants.hoodPosition
  }

  public void changeShooterUp() {
    shooterPower += 1;
  }
  public void changeShooterDown() {
    shooterPower -= 1;
  }

  public void changeHoodUp() {
    hoodPosition += 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));

  }

  public void hoodLow() {
      hood.setControl(m_motmag.withPosition(Constants.shooterConstants.hoodMinPosition));
  }
  public void changeHoodDown() {
    hoodPosition -= 0.05;
    hood.setControl(m_motmag.withPosition(hoodPosition));
  }


public static double hoodDegToMotorRot(double angleDeg) {
  double a1 = Constants.HoodConstants.ANGLE1_DEG;
  double p1 = Constants.HoodConstants.MOTOR_ROT1;
  double a2 = Constants.HoodConstants.ANGLE2_DEG;
  double p2 = Constants.HoodConstants.MOTOR_ROT2;

  // slope: motorRot per degree
  double rotPerDeg = (p2 - p1) / (a2 - a1);

  // motorRot = p1 + (angle - a1) * slope
  return p1 + (angleDeg - a1) * rotPerDeg;
}

public void setShooterRPM(double shooterRPM) 
  {
    shooterLeft.setControl(speedControl.withVelocity(shooterRPM));
    shooterRight.setControl(speedControl.withVelocity(shooterRPM));  
  }

public void setHoodAngleDegrees(double angleDeg) {
  double targetRot = hoodDegToMotorRot(angleDeg);

  // Clamp to physical limits
  targetRot = MathUtil.clamp(
      targetRot,
      Constants.HoodConstants.MIN_POS_ROT,
      Constants.HoodConstants.MAX_POS_ROT
  );
System.out.println("Angle degrees " + angleDeg);
  hood.setControl(m_motmag.withPosition(targetRot));
}

  public void mainHoodAngle() {
    if (finalHoodPosition.get(Constants.distToGoal) < shooterConstants.hoodMinPosition) {
      hood.setControl(m_motmag.withPosition(shooterConstants.hoodMinPosition));
    } else if (finalHoodPosition.get(Constants.distToGoal) > shooterConstants.hoodMaxPosition) {
      hood.setControl(m_motmag.withPosition(shooterConstants.hoodMaxPosition));
    } else {
      hood.setControl(m_motmag.withPosition(finalHoodPosition.get(Constants.distToGoal)));
      //hood.setControl(m_motmag.withPosition(mainHoodAngle));
    }
    //hood.setControl(m_motmag.withPosition(hoodAngleDegrees()));
  }

  public void setShooterToMathRPMFromMPS(double vSurface) {
      final double bottomRPM = (vSurface / Constants.ShotConstants.CIRC_BOTTOM_M) * 60.0;
      final double topRPM = (vSurface / Constants.ShotConstants.CIRC_TOP_M) * 60.0;    
      setDifferentShooterRPMs(bottomRPM, topRPM);
  }
  

  public void mainShooterPower() {
    shooterLeft.setControl(speedControl.withVelocity(finalShooterRPS.get(Constants.distToGoal)));
    shooterRight.setControl(speedControl.withVelocity(finalShooterRPS.get(Constants.distToGoal)));
    //shooterLeft.setControl(speedControl.withVelocity(shooterPower)); //CONSIDER ADDING A CONSTANT TO ALL OF THESE VALUES TO COMPENSATE FOR THE HEAVIER WEIGHT? 
    //shooterRight.setControl(speedControl.withVelocity(shooterPower));
    //shooterRight.set(finalShooterRPS.get(Constants.distToGoal));
  }

  public double hoodAngleDegrees() {
    return hoodAngleDegrees;
  }

  public double calculateTimeOfFlight(double distanceMeters, double shooterRPS, double hoodAngleDegrees) {
    if(vX <= 0) {
      calculatedTOF = 0;
    }
    else {
    calculatedTOF = distanceMeters/vX;
    }

    return calculatedTOF;
  }

  @Override
  public void periodic() {
    shooterRPS = shooterLeft.getVelocity().getValueAsDouble();


    mainHoodAngle = finalHoodPosition.get(Constants.distToGoal);
    mainShooterRPS = finalShooterRPS.get(Constants.distToGoal);

    SmartDashboard.putNumber("Shooter RPS Set To", shooterPower);
    SmartDashboard.putNumber("Real Shooter RPS ", shooterLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood Position ", hood.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood Angle Set to ", mainHoodAngle);
    SmartDashboard.putNumber("Shooter Power Set to ", mainShooterRPS);
    SmartDashboard.putNumber("Hood Set to ", hoodPosition);




    // SmartDashboard.putNumber("Shooter RPS ", shooterRight.getvelo);

    wheelSurfaceSpeed = shooterRPS * (Constants.shooterConstants.wheelDiameterMeters * Math.PI);
    exitVelocity = wheelSurfaceSpeed * Constants.shooterConstants.kSlip;
    hoodAngleRadians = Math.toRadians(hoodAngleDegrees);
    vX = exitVelocity * Math.cos(hoodAngleRadians);

    Constants.timeOfFlight = timeOfFlight.get(Constants.distToGoal);


    
    //System.out.println("shooter RPS" + shooterPower);
    //System.out.println("hood position " + hoodPosition);
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


    mainHoodAngle();


    //mainShooterPower();
  }

  
}

