// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {
  
  DigitalInput zeroTurret = new DigitalInput(0); //CHECK THIS DIO PORT! IT MIGHT BE WRONG!
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
  public static int shootMode = 4; //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.


  //Variables for getting angle to goal.
  double goalRedX = 12; //Red Goal
  double goalRedY = 4.034536;
  double goalBlueX = -12; //Red Goal
  double goalBlueY = 4.034536;
  double theta;
  double angleToGoal;
  double turretAngleTarget;
  double finalTurretPos;
  double vRobotY;
  double vRobotX;

  //FIELD LOCATIONS:

  //MAILING
  //RED LINE
  final double redLine = 11.5; //used to be 12.6, made it 11.5 for more accurate zone of when we want to do mailing funciton
  final double middleY = 4;
  final double blueLine = 4.1;

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

  Translation2d targetLocation;

  Rotation2d fieldTargetAngle;
  Rotation2d robotTargetAngle; 

  Translation2d goalLocation = new Translation2d(0,0);
  Translation2d targetVec;
  Translation2d vRobotPose;
  double dist, doubleDistance;

  double thetaX, thetaY;
  double toDegree;

  double timeOffset;
  private static final InterpolatingDoubleTreeMap hoodPosition = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterPower = new InterpolatingDoubleTreeMap();

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

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    config.Slot0 = Constants.slot0Configs;

    m_motmag.EnableFOC = true;
  
    Turret.getConfigurator().apply(config);
   
    // flightTime.put(1.0, 1.0);
    // flightTime.put(2.0, 1.5);
    // flightTime.put(3.0, 1.75);
    // flightTime.put(4.0, 2.0);
    // flightTime.put(5.0, 2.5);
  }

  public void rotateTurret(double turretSpeed) {
    Turret.set(turretSpeed);
  }

  public void stopTurret() {
    Turret.set(0);
  }

  public void setTurretPos(double pos) {
    Turret.setPosition(pos);
  }

  public void zeroTurret() {
    Turret.setPosition(-0.617187);
    System.out.println("Zero");
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
    SmartDashboard.putNumber("Shoot Mode ", shootMode);
    SmartDashboard.putNumber("Turret Angle ", finalTurretAngle);
    SmartDashboard.putNumber("Turret Position", Turret.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Turret Zero", zeroTurret.get());

    
    // vRobotY = drivetrain.getState().Pose.getY() - (drivetrain.getState().Speeds.vyMetersPerSecond * Constants.timeOfFlight) + 0.1;
    // vRobotX = drivetrain.getState().Pose.getX() - (drivetrain.getState().Speeds.vxMetersPerSecond * Constants.timeOfFlight);
    vRobotY = drivetrain.getState().Pose.getY() + (drivetrain.getState().Speeds.vyMetersPerSecond * (Constants.timeOfFlight * 1.2)) /*+ 0.2*/;
    vRobotX = drivetrain.getState().Pose.getX() + (drivetrain.getState().Speeds.vxMetersPerSecond * (Constants.timeOfFlight * 1.2)) /*- 0.2*/; //Added (Constants.timeOfFlight * 1.2) because the offset while driving wasn't enough

    robotPose = drivetrain.getState().Pose;
    robotPoseX = drivetrain.getState().Pose.getX();
    robotPoseY = drivetrain.getState().Pose.getY();

    if (red == true) {
      goalRedX = 12;
    } else {
      goalBlueX = 4;
    }
      // preTheta = (goalY - drivetrain.getState().Pose.getY()) / (goalX - drivetrain.getState().Pose.getX());
      // theta = Math.atan(preTheta);
    heading = drivetrain.getState().Pose.getRotation().getDegrees();
    theta = Math.atan2(thetaY, thetaX);
    toDegree = Math.toDegrees(theta);
    turretAngle = toDegree - heading;
    finalTurretAngle = edu.wpi.first.math.MathUtil.inputModulus(turretAngle, -90, 85);

    vRobotPose = new Translation2d(vRobotX, vRobotY);
    targetVec = goalLocation.minus(vRobotPose);
    dist = targetVec.getNorm();
    Constants.distToGoal = dist;
    //System.out.println("Dist to goal" + dist);


    //setTurretToAngle(finalTurretAngle);
    SmartDashboard.putNumber("Turret Position ", Turret.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Target Angle", turretAngle);
    SmartDashboard.putNumber("Final Turret Target Angle", finalTurretAngle);
    
    SmartDashboard.putBoolean("Zero Turret", zeroTurret.get());

      //System.out.println("Robot X" + drivetrain.getState().Pose.getX());

    
    switch (shootMode) {
      case 0:
        Turret.set(0);
        break;
      case 1:
        if (red == true) {
          goalLocation = new Translation2d(goalRedX, goalRedY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        } else {
          goalLocation = new Translation2d(goalBlueX, goalBlueY);
          thetaX = goalLocation.getX() - vRobotX;
          thetaY = goalLocation.getY() - vRobotY;
        }
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
          if (robotPoseX > redLine) {
            goalLocation = new Translation2d(goalRedX, goalRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY < 4 && robotPoseX < redLine) {//left (from red perspective)
            goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY > 4 && robotPoseX < redLine) { 
            goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
        }
        else { //blue
          if (robotPoseX < blueLine) {
            goalLocation = new Translation2d(goalBlueX, goalBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY < 4 && robotPoseX > blueLine) { //left (from red perspective)
            goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
          }
          if(robotPoseY > 4 && robotPoseX > blueLine) {
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
    SmartDashboard.putNumber("ShootMode: ", shootMode);

    //System.out.println("shoot mode:" + shootMode);
    // if (Turret.getPosition().getValueAsDouble() > 6.5 || Turret.getPosition().getValueAsDouble() < -6.5) {
    //   Turret.set(0);
    // }
    //System.out.println("Zero" + zeroTurret.get());
    if (zeroTurret.get() == false) {
      zeroTurret();
    }
  }
}
