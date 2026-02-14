// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.slot0Configs;

import java.lang.System.Logger;
import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.turretConstants;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {
  DigitalInput zeroTurret = new DigitalInput(turretConstants.turretID);
  //public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CommandSwerveDrivetrain drivetrain; 

  //Auto Aim To Hub 
  double latency = 0.15; // Tuned constant
  Translation2d futurePos;


  double idealHorizontalSpeed = 1; //double idealHorizontalSpeed = ShooterTable.getSpeed(dist);
  Translation2d robotVelVec;
  Translation2d shotVec;
  double finalTurretAngle;
  double newHorizontalSpeed;
  double ticksPerDegree;


  //double idealSpeed = getShooterSpeedForDistance(distance);
  

  boolean red;
  Optional<Alliance> alliance = DriverStation.getAlliance();
  public static int shootMode = 0; //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.


  //Variables for getting angle to goal.
  double goalX = 12; //Red Goal
  double goalY = 4.034536;
  double theta;
  double angleToGoal;
  double turretAngleTarget;
  double finalTurretPos;
  double vRobotY;
  double vRobotX;

  //FIELD LOCATIONS:

  //MAILING
  //RED LINE
  final double redLine = 12;
  final double middleX = 7.5;
  final double middleY = 4;
  final double blueLine = 3;

  //RED
  //RIGHT
  final double goalRightRedY = 7.211;
  final double goalRightRedX = 13.302;
  //LEFT
  final double goalLeftRedY = 0.8;
  final double goalLeftRedX = 13.302;

  //BLUE
  //RIGHT
  final double goalRightBlueY = 7.756;
  final double goalRightBlueX = 3.097;
  //LEFT
  final double goalLeftBlueY = 3;
  final double goalLeftBlueX = 0.8;



  double preTheta;

  Pose2d robotPose;
  double robotPoseX;
  double robotPoseY;

  //InterpolatingDoubleTreeMap turretPos = new InterpolatingDoubleTreeMap();
  double heading;
  double turretAngle;

  double testAngle;
  double angleToPos;

  Translation2d targetLocation = new Translation2d(goalX, goalY);

  Rotation2d fieldTargetAngle;
  Rotation2d robotTargetAngle; 

  Translation2d goalLocation = new Translation2d(goalX, goalY);
  Translation2d targetVec = goalLocation;
  double dist;

  double thetaX, thetaY;
  double toDegree;

  double timeOffset;
  private static final InterpolatingDoubleTreeMap flightTime = new InterpolatingDoubleTreeMap();

  public final static CommandXboxController driver = new CommandXboxController(0);
  
  /** Creates a new Turret. */
    private TalonFX Turret = new TalonFX(Constants.turretConstants.turretID);
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
    
    // private Encoder encoder = new Encoder(0, 0);
  // double dx;
  // double dy;
  // Rotation2d fieldAngle;

  public Turret(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    if (alliance.get() == Alliance.Red) {
      red = true;
    } else {
      red = false;
    }

    var talonFXConfigs = new TalonFXConfiguration();

    // talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Turret.getConfigurator().apply(config);

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


    Turret.getConfigurator().apply(motionMagicConfigs);
    Turret.getConfigurator().apply(slot0Configs); 


    // turretPos.put(0.0, -0.973633);
    // turretPos.put(90.0, 2.453125);
    // turretPos.put(180.0, 5.595215);
    // turretPos.put(270.0, 9.096680);
    flightTime.put(1.0, 1.0);
    flightTime.put(2.0, 1.5);
    flightTime.put(3.0, 1.75);
    flightTime.put(4.0, 2.0);
    flightTime.put(5.0, 2.5);
  }

  public void rotateTurret(double turretSpeed) {
    Turret.set(turretSpeed);
  }

  public void setTurretPos(double pos) {
    Turret.setPosition(pos);
  }

  public void zeroTurret() {
    Turret.setControl(m_motmag.withPosition(0));
  }

  public void fixedTurretPosition() {
    // heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
    // turretAngleTarget = 0 - heading;
    // finalTurretPos = turretAngleTarget * 13.2;
    // Turret.setControl(m_motmag.withPosition(finalTurretPos));
    // System.out.println(finalTurretPos);
  }


  public void turretPosition(double goalX, double goalY) {
      // theta = Math.atan((this.goalY - vRobotY) / (this.goalX - vRobotX));
      // angleToGoal = 90 - theta;
      // heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
      // turretAngleTarget = angleToGoal - heading;
      // finalTurretPos = turretAngleTarget * 13.2;
      // Turret.setControl(m_motmag.withPosition(finalTurretPos));
      // System.out.println(finalTurretPos);
  }

  public void shootModeChange(boolean up) {
    if (up == true) {
      shootMode += 1;
    } else {
      shootMode -= 1;
    }
  }

  public void setTurretToAngle (double angle) {
    angleToPos = ((angle /360) * 13.2);
    Turret.setControl(m_motmag.withPosition(angleToPos));
    //System.out.println("set Turret angle to" + angle);
  }

  public void manualTurret (double speed) {
    Turret.set(speed);
  }


  @Override
  public void periodic() {

    //System.out.println("shot mode " + shootMode);
    
    vRobotY = drivetrain.getState().Pose.getY() - (drivetrain.getState().Speeds.vyMetersPerSecond * 1) + 0.1;
    vRobotX = drivetrain.getState().Pose.getX() - (drivetrain.getState().Speeds.vxMetersPerSecond * 1);

    robotPose = drivetrain.getState().Pose;
    robotPoseX = drivetrain.getState().Pose.getX();
    robotPoseY = drivetrain.getState().Pose.getY();

    if (red == true) {
      goalX = 12;
    } else {
      goalX = 4;
    }
      // preTheta = (goalY - drivetrain.getState().Pose.getY()) / (goalX - drivetrain.getState().Pose.getX());
      // theta = Math.atan(preTheta);
    heading = drivetrain.getState().Pose.getRotation().getDegrees();
    theta = Math.atan2(thetaY, thetaX);
    toDegree = Math.toDegrees(theta);
    turretAngle = toDegree - heading;
    finalTurretAngle = edu.wpi.first.math.MathUtil.inputModulus(turretAngle, -180, 180);

    //setTurretToAngle(finalTurretAngle);
      //System.out.println("Robot X" + drivetrain.getState().Pose.getX());

    
    switch (shootMode) {
      case 1:
        goalLocation = new Translation2d(goalX, goalY);
        thetaX = goalLocation.getX() - vRobotX;
        thetaY = goalLocation.getY() - vRobotY;
        break;
    
      case 2:
        if (red == true) {
          goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        } else {
          goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        }
        break;

      case 3:
        if (red == true) {
          goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        } else {
          goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        }
        break;
      case 4: //red 
        if(red == true) {
          if(robotPoseY < 3) {//left (from red perspective)
            goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY > 3) { 
            goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
        }
        else { //blue
          if(robotPoseY < 3) { //left (from red perspective)
            goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY > 3) {
            goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          
        }


      
        
    }


    if (shootMode > 4) {
      shootMode = 4;
    } else if (shootMode < 0) {
      shootMode = 0;
    }
    //System.out.println("shoot mode:" + shootMode);
    // if (Turret.getPosition().getValueAsDouble() > 6.5 || Turret.getPosition().getValueAsDouble() < -6.5) {
    //   Turret.set(0);
    // }
    //System.out.println("Zero" + zeroTurret.get());
    if (zeroTurret.get() == false) {
      System.out.println("false");
      zeroTurret();


    }
  }
}
