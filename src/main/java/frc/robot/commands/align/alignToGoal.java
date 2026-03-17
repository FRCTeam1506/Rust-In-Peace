//the improper way to do this, just putting the values from the limelight straight to the drivetrain.
//if it works, it works -- 1506

package frc.robot.commands.align;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;

public class alignToGoal extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final ProfiledPIDController thetaController;

    boolean isFinished = false;

    double wantedHeading;
    double heading;

    double toRadians;

    private final SwerveRequest.FieldCentric m_alignRequest;

    public final CommandPS4Controller driver = new CommandPS4Controller(0);
    double xSpeed;
    double ySpeed;

    public alignToGoal(double wantedHeading, CommandSwerveDrivetrain drivetrain) {
        this.wantedHeading = wantedHeading;
        this.drivetrain = drivetrain;
        
        this.m_alignRequest = new SwerveRequest.FieldCentric();
        
        thetaController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(2, 2));

        thetaController.enableContinuousInput(-180, 180); 

        isFinished = false;
    }

    @Override
    public void initialize() {
        isFinished = false;
        thetaController.setGoal(0); // tx=0 is centered

    }
    

    @Override
    public void execute() {
        wantedHeading = Turret.theta;
        heading = drivetrain.getState().Pose.getRotation().getDegrees();

        xSpeed = -driver.getLeftY() * RobotContainer.MaxSpeed;
        ySpeed = -driver.getLeftX() * RobotContainer.MaxSpeed;
    
        double rotationOutput = thetaController.calculate(heading, wantedHeading);

        toRadians = Math.toRadians(rotationOutput);

        drivetrain.setControl(m_alignRequest
            .withVelocityX(xSpeed)//forwards and backwards? YES
            .withVelocityY(ySpeed) 
            .withRotationalRate(toRadians));
        

    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(m_alignRequest
            .withVelocityX(0)//forwards and backwards? YES
            .withVelocityY(0) 
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return thetaController.atGoal(); //aimController.atGoal();
        // return thetaController.atGoal() && drivetrain.getState().Speeds.vxMetersPerSecond < 0.1;
    }
}