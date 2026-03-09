// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  //HOOD ENCODER PID CONTROLLER SETUP:
  double xVelocity;
  private final ProfiledPIDController xController =
  new ProfiledPIDController(
      25,
      SwerveConstants.driveKI,
      SwerveConstants.driveKD,
      new TrapezoidProfile.Constraints(5, SwerveConstants.dMaxAccel),
      0.02);
  //SHOOTER CONTROLLER SETUP: (WAS USED TO TRY AND MAKE SHOOTER RPM LESS VARIABLE):
  // private final ProfiledPIDController shooterController =
  // new ProfiledPIDController(
  //     200,
  //     1,
  //     1,
  //     new TrapezoidProfile.Constraints(5, SwerveConstants.dMaxAccel),
  //     0.02);
  private TalonFX shooterLeft = new TalonFX(shooterConstants.shooterLeft);
  private TalonFX shooterRight = new TalonFX(shooterConstants.shooterRight);
  private TalonSRX hood = new TalonSRX(16);
  private CANcoder hoodEncoder = new CANcoder(55);

  double shooterPower = 50;
  double hoodPosition = 0.0;

  public static double shooterRPS;
  public static double wheelSurfaceSpeed;
  public static double exitVelocity;
  public static double hoodAngleDegrees;
  public static double hoodAngleRadians;
  public static double vX;
  public static double calculatedTOF;
  public double setPoint;
  public boolean toggleManualHood;

  final VelocityVoltage speedControl = new VelocityVoltage(12);


  //FOR ALL OF THESE, KEY IS DISTANCE, OUTPUT IS NAME OF THE TABLE
  public InterpolatingDoubleTreeMap finalHoodPosition = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap finalShooterRPS = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();


  double mainShooterRPS, mainHoodAngle;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Intake. */
  public Shooter() {
    // Sets distance conversion: (Wheel Diameter * PI) / PulsesPerRevolution
    //m_encoder.setDistancePerPulse(1.0 / 360.0);
    hood.configRemoteFeedbackFilter(55, RemoteSensorSource.CANCoder,0);
    hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    hood.config_kP(0, 1, 10); // Adjust PID values
    hood.config_kI(0, 1, 10);
    hood.config_kD(0, 0, 10);
    hood.config_kF(0, 0.0, 10);
    hood.configMotionCruiseVelocity(1);
    hood.configMotionAcceleration(0.2);
    hood.setControlFramePeriod(ControlFrame.Control_3_General, 100);

    var talonFXConfigs = new TalonFXConfiguration();
    TalonSRXConfiguration SRXconfig = new TalonSRXConfiguration();

    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;


    //talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    var slot0SRXconfig = SRXconfig.slot0;

    slot0SRXconfig.kP = 2.5; //2.5
    slot0SRXconfig.kI = 0;
    slot0SRXconfig.kD = 0.25;

    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction //0.24
    //slot0Configs.kV = 0.004; // apply 12 V for a target velocity of 100 rps //0.12
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps //0.12

    // PID runs on position
    slot0Configs.kP = 100; //2.5
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.25;

    SRXconfig.slot0 = slot0SRXconfig;
  
    m_motmag.EnableFOC = true;

    // hood.getConfigurator().apply(motionMagicConfigs);
    // hood.getConfigurator().apply(slot0hoodConfigs); 
    hood.configAllSettings(SRXconfig);
    shooterLeft.getConfigurator().apply(slot0Configs);
    shooterRight.getConfigurator().apply(slot0Configs);
    //hood.getConfigurator().apply(config);

    finalHoodPosition.put(1.35, shooterConstants.hoodMinPosition); //somewhat close to the hub
    finalHoodPosition.put(2.54, shooterConstants.hoodMinPosition + 0.03); //-0.005 old //at the climbing rack
    finalHoodPosition.put(2.86, shooterConstants.hoodMinPosition + 0.045); //0.01 old //at the back wall
    finalHoodPosition.put(3.98, shooterConstants.hoodMinPosition + 0.065); //0.03 old //back against the back wall
    finalHoodPosition.put(4.11, shooterConstants.hoodMinPosition + 0.07); //0.035 //in the square thing
    finalHoodPosition.put(3.15, shooterConstants.hoodMinPosition + 0.02); 
    //finalHoodPosition.put(5.03, -1.66796875); //in the back corner
    //finalHoodPosition.put(3.23, -0.66); //auton starting point

    finalShooterRPS.put(1.35, 51.0);
    finalShooterRPS.put(2.54, 78.0);
    finalShooterRPS.put(2.86, 90.0);
    finalShooterRPS.put(3.98, 90.0);// consider lowering because it is overshooting
    finalShooterRPS.put(4.11, 95.0);
    finalShooterRPS.put(3.15, 62.0);

    //finalShooterRPS.put(5.03, 83.0);
    //finalShooterRPS.put(3.23, 70);

    timeOfFlight.put(1.35, 0.95);
    timeOfFlight.put(2.54, 1.15);
    timeOfFlight.put(2.86, 1.2);
    timeOfFlight.put(3.98, 1.3);
    timeOfFlight.put(4.11, 1.2);
    //timeOfFlight.put(5.03, 1.55);
    //timeOfFlight.put(3.23, 1.55);
  }

  public void shoot(double speed) {
    //shooterLeft.set(shooterPower/100); //manual adjust
    //shooterRight.set(shooterPower/100); manual adjust
    //shooterLeft.set((shooterController.calculate(shooterLeft.getRotorVelocity().getValueAsDouble(), shooterPower)));
    //shooterRight.set(-(shooterController.calculate(shooterLeft.getRotorVelocity().getValueAsDouble(), shooterPower)));

    //shooterLeft.setControl(speedControl.withVelocity(-shooterPower)); //auto power
    //shooterRight.setControl(speedControl.withVelocity(shooterPower)); //auto power
    shooterLeft.setControl(speedControl.withVelocity(-finalShooterRPS.get(Constants.distToGoal))); //auto power
    shooterRight.setControl(speedControl.withVelocity(finalShooterRPS.get(Constants.distToGoal))); //auto power
  }

  public void stopShooter() {
    shooterLeft.set(0); 
    shooterRight.set(0);
  }

  public void manualShooterSPEED(double speed) {
    shooterLeft.set(-speed);
    shooterRight.set(speed);
  }

  public void shootSpeed(double speed) {
    shooterLeft.setControl(speedControl.withVelocity(speed));
    shooterRight.setControl(speedControl.withVelocity(speed));
  }
  public void zeroHood() {
    hoodEncoder.setPosition(0);
  }

  public void setDifferentShooterRPMs(double bottomRPM, double topRPM) {
    shooterLeft.setControl(speedControl.withVelocity(bottomRPM/60)); //MAKE SURE THAT BOTTOM IS "LEFT" AND TOP IS "RIGHT" 
    shooterRight.setControl(speedControl.withVelocity(topRPM/60));
  }

  public void hood (double speed) {
    hood.set(ControlMode.PercentOutput, speed);
  }

  public void setHood(double position) {
    hood.set(TalonSRXControlMode.MotionMagic, position); //was position, then Constants.shooterConstants.hoodPosition
    
  }

  public void changeShooterUp() {
    shooterPower += 1;
  }
  public void changeShooterDown() {
    shooterPower -= 1;
  }

  public void changeHoodUp() {
    hoodPosition += 0.005;
  }

  public void changeHoodDown() {
  
    hoodPosition -= 0.005;
  }

  public void hoodLow() {
      //hood.set(ControlMode.MotionMagic, shooterConstants.hoodMinPosition);
      toggleManualHood = true;
      hoodPosition = shooterConstants.hoodMinPosition;
  }

  public void automaticHood() {
    toggleManualHood = false;
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
    shooterLeft.setControl(speedControl.withVelocity(-shooterRPM));
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
  //hood.setControl(m_motmag.withPosition(targetRot));
}

  public void mainHoodAngle() {
    // if (finalHoodPosition.get(Constants.distToGoal) < shooterConstants.hoodMaxPosition) { //
    //   //hood.set(ControlMode.Position, shooterConstants.hoodMinPosition);
    //   hoodPosition = shooterConstants.hoodMinPosition;
    // } else if (finalHoodPosition.get(Constants.distToGoal) > shooterConstants.hoodMinPosition) {
    //   //hood.set(ControlMode.Position, shooterConstants.hoodMaxPosition);
    //   hoodPosition = shooterConstants.hoodMaxPosition;
    // } else {
    //   hood.set(ControlMode.Position, finalHoodPosition.get(Constants.distToGoal));
    //   hoodPosition = finalHoodPosition.get(Constants.distToGoal);
    //   //hood.setControl(m_motmag.withPosition(mainHoodAngle));
      
    // }
    //hood.setControl(m_motmag.withPosition(hoodAngleDegrees()));
  }

  public void setShooterToMathRPMFromMPS(double vSurface) {
      final double bottomRPM = (vSurface / Constants.ShotConstants.CIRC_BOTTOM_M) * 60.0;
      final double topRPM = (vSurface / Constants.ShotConstants.CIRC_TOP_M) * 60.0;    
      setDifferentShooterRPMs(bottomRPM, topRPM);
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
  

  public void mainShooterPower() {
    shooterLeft.setControl(speedControl.withVelocity(-finalShooterRPS.get(Constants.distToGoal)));
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

  private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Voltage.ofBaseUnits(4, Volt), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
         // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> shooterLeft.setControl(m_voltReq.withOutput(volts.in(Volt))),
         null,
         this
      )
   );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
    }

  @Override
  public void periodic() {
    if(toggleManualHood == true) {
    }

    if(toggleManualHood == false) {
    
    //mainHoodAngle();
      if (finalHoodPosition.get(Constants.distToGoal) > shooterConstants.hoodMaxPosition) { //
        //hood.set(ControlMode.Position, shooterConstants.hoodMinPosition);
        hoodPosition = shooterConstants.hoodMinPosition;
      } else if (finalHoodPosition.get(Constants.distToGoal) < shooterConstants.hoodMinPosition) {
        //hood.set(ControlMode.Position, shooterConstants.hoodMaxPosition);
        hoodPosition = shooterConstants.hoodMaxPosition;
      } else {
        //hood.set(ControlMode.Position, finalHoodPosition.get(Constants.distToGoal));
        hoodPosition = finalHoodPosition.get(Constants.distToGoal);
        //hood.setControl(m_motmag.withPosition(mainHoodAngle));
      }
    }

    setPoint = hoodEncoderPosition(hoodPosition);
    hood.set(ControlMode.PercentOutput, setPoint);
    System.out.println("hood power set to pid value: " + setPoint);
    SmartDashboard.putNumber("Main Hood Position With PID", setPoint);
    SmartDashboard.putBoolean("Toggle Manual Hood", toggleManualHood);



    //add lower hood if going under trench
    shooterRPS = shooterLeft.getVelocity().getValueAsDouble();


    mainHoodAngle = finalHoodPosition.get(Constants.distToGoal);
    mainShooterRPS = finalShooterRPS.get(Constants.distToGoal);

    SmartDashboard.putNumber("Shooter RPS Set To", shooterPower);
    SmartDashboard.putNumber("Real Shooter RPS ", shooterLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood Position ", hoodEncoder.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Hood Motor? Position ", hood.getSelectedSensorPosition());

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




    //mainShooterPower();
  }

  
}