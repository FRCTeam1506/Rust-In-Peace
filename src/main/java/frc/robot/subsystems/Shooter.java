// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {

  //motor and encoder setup:
  private TalonFX shooterLeft = new TalonFX(shooterConstants.shooterLeftID);
  private TalonFX shooterRight = new TalonFX(shooterConstants.shooterRightID);
  private TalonSRX hood = new TalonSRX(16);
  private CANcoder hoodEncoder = new CANcoder(55);

  //HOOD ENCODER PID CONTROLLER SETUP:
  double xVelocity;
  private final ProfiledPIDController xController =
  new ProfiledPIDController(
      25,
      0.15,
      0.175,
      new TrapezoidProfile.Constraints(5, 2.0),
      0.02);
  
  //SHOOTING VARIABLES:
  double shooterPower = 50;
  double manualShootingRPS = 50.0; //Used to get points or just in case

  //HOOD VARIABLES:
  double hoodPosition = 0.0; //Position that hood is set to

  public double setPoint;
  public boolean toggleManualHood;

  
  final VelocityVoltage speedControl = new VelocityVoltage(12);
  final VelocityTorqueCurrentFOC speedControl2 = new VelocityTorqueCurrentFOC(0).withSlot(1); //TEST THIS

  //FOR ALL OF THESE, KEY IS DISTANCE, OUTPUT IS NAME OF THE TABLE
  //LUTS:
  public InterpolatingDoubleTreeMap finalHoodPosition = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap finalShooterRPS = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

  public double mainShooterRPS;
  public double mainHoodAngle;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Intake. */
  public Shooter() {
    //initDefaultCommand();
    //HOOD CONFIGURATION:
    //m_encoder.setDistancePerPulse(1.0 / 360.0);
    hood.configRemoteFeedbackFilter(55, RemoteSensorSource.CANCoder,0);
    hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    hood.config_kP(0, 3, 10); // Adjust PID values
    hood.config_kI(0, 1, 10);
    hood.config_kD(0, 0, 10);
    hood.config_kF(0, 0.0, 10);
    hood.configMotionCruiseVelocity(1);
    hood.configMotionAcceleration(0.2);
    hood.setControlFramePeriod(ControlFrame.Control_3_General, 100);


    //SHOOTER CONFIGURATION:
    var shooterConfigs = new TalonFXConfiguration();

    shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfigs.CurrentLimits.StatorCurrentLimit = 80;

    var motionMagicConfigs = shooterConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = shooterConfigs.Slot0;
    
    slot0Configs.kS = 2.5;//0.24 // add 0.24 V to overcome friction //0.24
    slot0Configs.kV = 0;//0.12 // apply 12 V for a target velocity of 100 rps //0.12
    slot0Configs.kP = 5;//100 //2.5
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;//0.25
  
    m_motmag.EnableFOC = true;

    shooterLeft.getConfigurator().apply(shooterConfigs);
    shooterRight.getConfigurator().apply(shooterConfigs);

    //LOOKUP TABLES:
    finalHoodPosition.put(1.42, shooterConstants.hoodMinPosition);
    finalHoodPosition.put(1.91, shooterConstants.hoodMinPosition); //somewhat close to the hub
    finalHoodPosition.put(2.54, shooterConstants.hoodMinPosition + 0.02); //-0.005 old //at the climbing rack
    finalHoodPosition.put(3.25, shooterConstants.hoodMinPosition + 0.035);
    finalHoodPosition.put(3.67, shooterConstants.hoodMinPosition + 0.05); //0.01 old //at the back wall

    finalHoodPosition.put(6.0, shooterConstants.hoodMaxPosition); //Used to be 5.2, changed it after measuring the actual distance in pathplanner//Full field
    // finalHoodPosition.put(3.25, shooterConstants.hoodMinPosition + 0.035); //0.03 old //back against the back wall
    // finalHoodPosition.put(4.11, shooterConstants.hoodMinPosition + 0.07); //0.035 //in the square thing
    // finalHoodPosition.put(3.15, shooterConstants.hoodMinPosition + 0.02); 
    //finalHoodPosition.put(5.03, -1.66796875); //in the back corner
    //finalHoodPosition.put(3.23, -0.66); //auton starting point


    finalShooterRPS.put(1.42, 46.0);
    finalShooterRPS.put(1.91, 54.0);
    finalShooterRPS.put(2.54, 65.0);
    finalShooterRPS.put(3.25, 82.0);
    finalShooterRPS.put(3.67, 91.0);
    finalShooterRPS.put(5.2, 95.0); //Full field
    // finalShooterRPS.put(3.25, 82.0);// consider lowering because it is overshooting
    // finalShooterRPS.put(4.11, 95.0);
    // finalShooterRPS.put(3.15, 62.0);
    //finalShooterRPS.put(5.03, 83.0);
    //finalShooterRPS.put(3.23, 70);

    timeOfFlight.put(1.42, 1.08);
    timeOfFlight.put(1.91, 1.12);
    timeOfFlight.put(2.54, 1.2);
    timeOfFlight.put(3.25, 1.25);
    timeOfFlight.put(3.98, 1.2);
    timeOfFlight.put(5.2, 1.75); //Full field
    // timeOfFlight.put(4.11, 1.2);
    //timeOfFlight.put(5.03, 1.55);
    //timeOfFlight.put(3.23, 1.55);
  }

  public void shoot() {
    //shooterLeft.set(shooterPower/100); //MANUAL POWER
    //shooterRight.set(shooterPower/100); //MANUAL POWER

    //shooterLeft.setControl(speedControl.withVelocity(-shooterPower)); //MANUAL RPS
    //shooterRight.setControl(speedControl.withVelocity(shooterPower)); //MANUAL RPS
    if (finalHoodPosition.get(Constants.distToGoal) > shooterConstants.hoodMaxPosition) { //
        hoodPosition = shooterConstants.hoodMinPosition;
      } else if (finalHoodPosition.get(Constants.distToGoal) < shooterConstants.hoodMinPosition) {
        hoodPosition = shooterConstants.hoodMaxPosition;
      } else {
        hoodPosition = finalHoodPosition.get(Constants.distToGoal);
      }
    shooterLeft.setControl(speedControl.withVelocity(-finalShooterRPS.get(Constants.distToGoal))); //AUTO POWER
    shooterRight.setControl(speedControl.withVelocity(finalShooterRPS.get(Constants.distToGoal))); //AUTO POWER
  }

  public void incrementalShooter() {
     shooterLeft.setControl(speedControl.withVelocity(-manualShootingRPS)); //MANUAL RPS
     shooterRight.setControl(speedControl.withVelocity(manualShootingRPS));
  }

  public void manualShooter(double rps) {
     shooterLeft.setControl(speedControl.withVelocity(-rps)); //MANUAL RPS
     shooterRight.setControl(speedControl.withVelocity(rps));
  }

  public void setHood(double position) {
    hood.set(TalonSRXControlMode.MotionMagic, position); //was position, then Constants.shooterConstants.hoodPosition
  }

  //MANUAL SHOOTER:
  public void changeShooterUp() {
    manualShootingRPS += 1;
  }
  public void changeShooterDown() {
    manualShootingRPS -= 1;
  }

  //MANUALHOOD:
  public void changeHoodUp() {
    hoodPosition += 0.005;
  }

  public void changeHoodDown() {
  
    hoodPosition -= 0.005;
  }


  public void automaticHood() {
    toggleManualHood = false;
  }

  public void manualHood() {
    toggleManualHood = true;
  }
  
  public double hoodEncoderPosition(double targetPosition) {
    if(DriverStation.isEnabled()) {
    double targetSpeed = -xController.calculate(hoodEncoder.getPosition().getValueAsDouble(), targetPosition);
    return Math.min(targetSpeed, 0.5);
    }
    return 0.0;
  }

  public void runHood(double speed) {
    hood.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopHood() {
    hood.set(ControlMode.PercentOutput, 0);
  }

  public void stopShooter() {
    shooterLeft.set(0); 
    shooterRight.set(0);
  }

  public void zeroHood() {
    hoodEncoder.setPosition(0);
  }

  public void hoodLow() {
      //hood.set(ControlMode.MotionMagic, shooterConstants.hoodMinPosition);
      System.out.println("RUNNING HOOD LOW!");
      toggleManualHood = true;
      hoodPosition = shooterConstants.hoodMinPosition;
  }

  //GET THIS TO WORK!
  // public void initDefaultCommand() {
  //       setPoint = hoodEncoderPosition(Constants.shooterConstants.hoodMinPosition);
  //       setDefaultCommand(new InstantCommand(() -> hood.set(ControlMode.PercentOutput, setPoint)));
  // }
  public void endShooter() {
  // Set motors to 0
  shooterLeft.set(0);
  shooterRight.set(0);
  // Set the hood target to your "down" position
  hoodPosition = shooterConstants.hoodMinPosition;
}

  @Override
  public void periodic() {

    if(toggleManualHood == true) {

    }
    else{
      if (finalHoodPosition.get(Constants.distToGoal) > shooterConstants.hoodMaxPosition) { //take all of this out when trying to get defualt command to work
        hoodPosition = shooterConstants.hoodMinPosition;
      } else if (finalHoodPosition.get(Constants.distToGoal) < shooterConstants.hoodMinPosition) {
        hoodPosition = shooterConstants.hoodMaxPosition;
      } else {
        hoodPosition = finalHoodPosition.get(Constants.distToGoal);
      }
    }
    

    Constants.shooterConstants.hoodPosition = hoodPosition;
    setPoint = hoodEncoderPosition(hoodPosition); //take this out when trying to get defualt command to work
    hood.set(ControlMode.PercentOutput, setPoint); //take this out when trying to get defualt command to work

    //add lower hood if going under trench
    

    //SHOOTER DASHBOARD VALUES:
    SmartDashboard.putNumber("MANUAL Shooter RPS Set To", shooterPower);
    SmartDashboard.putNumber("Real Shooter RPS", shooterLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Power Set to ", finalShooterRPS.get(Constants.distToGoal));

    //HOOD DASHBOARD VALUES:
    SmartDashboard.putBoolean("Toggle Manual Hood", toggleManualHood);
    SmartDashboard.putNumber("Hood Power BeingSet To", setPoint);
    SmartDashboard.putNumber("Hood Position Set to ", hoodPosition);
    SmartDashboard.putNumber("Hood Position At", hoodEncoder.getPosition().getValueAsDouble());


    Constants.timeOfFlight = timeOfFlight.get(Constants.distToGoal);
  } 
}