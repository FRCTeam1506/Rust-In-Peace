// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

// import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.pathplanner.lib.auto.AutoBuilder;

public class Robot extends TimedRobot {

  private final boolean kUseLimelight = true;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;



  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 


    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      double yawRateDegPerSec = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
      m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 999999));

      
      // LimelightHelpers.SetRobotOrientation(VisionConstants.LL_LEFT, headingDeg, 0, 15, 0, 180, 0);
      // // LimelightHelpers.SetRobotOrientation(VisionConstants.LL_RIGHT, headingDeg, 0, 15, 0, 180, 0);

      // LimelightHelpers.SetRobotOrientation(VisionConstants.LL_LEFT, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
      // LimelightHelpers.SetRobotOrientation(VisionConstants.LL_RIGHT, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);

      LimelightHelpers.SetRobotOrientation(VisionConstants.LL_LEFT, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(VisionConstants.LL_RIGHT, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      
      var llMeasurement_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_LEFT);
      var llMeasurement_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_RIGHT);

      if (llMeasurement_left != null && llMeasurement_left.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_LEFT) > 0.33) {

        // Pose2d pose = new Pose2d(llMeasurement.pose.getX(), llMeasurement.pose.getY(), llMeasurement.pose.getRotation().minus(new Rotation2d(0))); //minus rotation2d(math.pi)
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_left.pose, llMeasurement_left.timestampSeconds);
        //System.out.println("left " + llMeasurement_left.pose);
        System.out.println("heading " + headingDeg);

        // SmartDashboard.putNumberArray("MT2Result_Center", new double[]{llMeasurement.pose.getX(), llMeasurement.pose.getY()});
      }

      if (llMeasurement_right != null && llMeasurement_right.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_RIGHT) > 0.33) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_right.pose, llMeasurement_right.timestampSeconds);
        //System.out.println("right " + llMeasurement_right.pose);
        System.out.println("heading " + headingDeg);

      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void robotInit(){

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
public void simulationPeriodic() {
      // Obtains the default instance of the simulation world, which is a Crescendo Arena.
  // Overrides the default simulation
  // SimulatedArena.getInstance();
  // SimulatedArena.getInstance().simulationPeriodic();
  
    if (RobotBase.isSimulation() && m_robotContainer.getDrivetrain() != null) {

        // This line is crucial for the simulation to stay open
        m_robotContainer.getDrivetrain().simulationPeriodic(); 
        
    }
}
  // @Override
  // public void simulationPeriodic() {
  //   if (RobotBase.isSimulation() && m_robotContainer.getDrivetrain() != null) {
  //           // This is where you might call specific vendor simulation updates
  //           // Example (check your library documentation for exact call):
  //           // m_robotContainer.getDrivetrain().updateSimState(); 
  //       }
  // }
}