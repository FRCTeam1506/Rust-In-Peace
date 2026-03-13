// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;

public class Turret extends SubsystemBase {

    private TalonFX turret = new TalonFX(turretConstants.turretID);
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
    DigitalInput zeroTurret = new DigitalInput(0);

    private final CommandSwerveDrivetrain drivetrain;

    public static int shootMode = 1;
    boolean red;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    //Positions
    //Variables for getting angle to goal.
    double goalRedX = 12; //Red Goal
    double goalRedY = 4.034536;
    double goalBlueX = -12; //Blue Goal
    double goalBlueY = 4.034536;
    Translation2d goalLocation = new Translation2d(0, 0);

    //FIELD LOCATIONS:
    final double redLine = 11.5; //used to be 12.6, made it 11.5 for more accurate zone of when we want to do mailing funciton
    final double middleY = 4;
    final double blueLine = 4.1;

    final double goalRightRedY = 6;//7.211
    final double goalRightRedX = 13.302;

    final double goalLeftRedY = 2; //0.8
    final double goalLeftRedX = 13.302;

    final double goalRightBlueY = 6; //7.756
    final double goalRightBlueX = 3.097;

    final double goalLeftBlueY = 2; //3
    final double goalLeftBlueX = 0.8;


    //Math variables
    double theta;
    double thetaX, thetaY;
    double angleToGoal;
    double toDegree;
    double turretAngleTarget;
    double angleToPos;
    double turretAngle;
    double dist;
    double finalTurretAngle;
    Translation2d targetVec;

    //Robot Poses
    Pose2d robotPose;
    double robotPoseX;
    double robotPoseY;
    double omega;
    double heading;

    Translation2d rotationalVelocityField;

    //Virtual Robot Poses
    double vRobotY;
    double vRobotX;
    double vRotationalRobotY;
    double vRotationalRobotX;
    double totalFieldVy;
    double totalFieldVx;
    Translation2d vRobotPose;
    
    //Turret offset
    Transform2d turretOffset = new Transform2d(-0.19, -0.14, new Rotation2d(0));
    Pose2d turretPose = new Pose2d();

    //Extra
    double flightTimeMultiplier = 1.3; //If turret doesn't offset enough. Tune this if needed


  public Turret(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    if (alliance.get() == Alliance.Red) {
      red = true;
    } else {
      red = false;
    }

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    config.Slot0 = turretConstants.slot0Configs;

    m_motmag.EnableFOC = true;
  
    turret.getConfigurator().apply(config);
  }



  //Non-Important Methods
  public void rotateTurret(double turretSpeed) {
    turret.set(turretSpeed);
  }
  // public void manualTurret (double speed) {
  //   if (Constants.manualTurret = true) {
  //     turret.set(speed);
  //   }
  // }
  public void stopTurret() {
    turret.set(0);
  }

  //Main Methods
  public void zeroTurret() {
    turret.setPosition(-0.525); //-0.6171807, was playing with this
    System.out.println("Zero");
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
    turret.setControl(m_motmag.withPosition(angleToPos));
  }



  @Override
  public void periodic() {
    //Robot Pose and velocity
    omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    robotPose = drivetrain.getState().Pose;
    robotPoseX = drivetrain.getState().Pose.getX();
    robotPoseY = drivetrain.getState().Pose.getY();

    // if(robotPoseX > 4.1 && robotPoseX < 5.2) {
    //   Constants.shooterConhoodPosition = 
    // }
    

    //Use this instead of drivetrain pose to account for tuuret offset and rotation of the robot. 
    turretPose = drivetrain.getState().Pose.transformBy(turretOffset); 

    //Virtual rotational pose
    vRotationalRobotY = omega * turretOffset.getX();
    vRotationalRobotX = -omega * turretOffset.getY();
    rotationalVelocityField = new Translation2d(vRotationalRobotX, vRotationalRobotY).rotateBy(drivetrain.getState().Pose.getRotation());

    //Add speed
    totalFieldVy = drivetrain.getState().Speeds.vyMetersPerSecond + rotationalVelocityField.getY();
    totalFieldVx = drivetrain.getState().Speeds.vxMetersPerSecond + rotationalVelocityField.getX();
    
    //Times time of flight
    vRobotY = turretPose.getY() + (totalFieldVy * Constants.timeOfFlight * flightTimeMultiplier);
    vRobotX = turretPose.getX() + (totalFieldVx * Constants.timeOfFlight * flightTimeMultiplier);
    vRobotPose = new Translation2d(vRobotX, vRobotY);

    //Math to get angle 
    heading = drivetrain.getState().Pose.getRotation().getDegrees();
    theta = Math.atan2(thetaY, thetaX);
    toDegree = Math.toDegrees(theta);
    turretAngle = toDegree - heading;

    //Set min and max
    finalTurretAngle = edu.wpi.first.math.MathUtil.inputModulus(turretAngle, -90, 85);

    //Set turret to angle if auto aim is being used
    if (shootMode > 0) {
      setTurretToAngle(finalTurretAngle);
    } else {
      
    }

    //Get dist to goal
    targetVec = goalLocation.minus(vRobotPose);
    dist = targetVec.getNorm();
    Constants.distToGoal = dist;
    //System.out.println("Dist to goal" + dist);

    //Shoot mode
    switch (shootMode) {
        case 0: //Keep turret at zero
          //turret.setControl(m_motmag.withPosition(0));
          //Constants.manualTurret = true;
          break;
        case 1: //Change goal based on robot position
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
            //Constants.manualTurret = false;
            break;
        case 2: //Goal only
            if (red == true) {
                goalLocation = new Translation2d(goalRedX, goalRedY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
            } else {
                goalLocation = new Translation2d(goalBlueX, goalBlueY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
            }
            //Constants.manualTurret = false;
            break;
        case 3: //Aim to the left side for mailing DRIVER PERSPECTIVE
            if (red == true) {
            goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
            } else {
            goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
            }
            //Constants.manualTurret = false;
            break;
        case 4: //Aim to the right side for mailing DRIVER PERSPECTIVE
            if (red == true) {
            goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
            } else {
            goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
            thetaX = goalLocation.getX() - vRobotX;
            thetaY = goalLocation.getY() - vRobotY;
            }
            //Constants.manualTurret = false;
            break;
    
        default: //Anything else stop turret
            turret.set(0);
            //Constants.manualTurret = true;
            break;
    }

    //Print out info
    SmartDashboard.putNumber("Turret Position ", turret.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Target Angle", turretAngle);
    SmartDashboard.putNumber("Constrained Turret Target Angle", finalTurretAngle);
    SmartDashboard.putBoolean("Zero Turret", zeroTurret.get());
    SmartDashboard.putNumber("Shoot Mode ", shootMode);
    SmartDashboard.putNumber("Dist to goal", dist);
    //SmartDashboard.putNumber("Turret Angle Set To", finalTurretAngle); 
  
  
    //Zero turret if sensor is triggered
    if (zeroTurret.get() == false) {
      zeroTurret();
    }

    //Check red vs blue alliance and change for different targets
    if (red == true) {
      goalRedX = 12;
    } else {
      goalBlueX = 4;
    }

    //Set shoot mode in constants for shooter subsytem to get
    //Constants.shootMode = shootMode;
  }
}