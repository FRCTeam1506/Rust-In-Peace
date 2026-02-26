package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.ShootingSolver;

public class ShootOnTheMove extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Turret turret;

    private double lastToF = 0.5;

    public ShootOnTheMove(CommandSwerveDrivetrain drivetrain, Shooter shooter, Turret turret) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;

        // We require shooter + turret because we're commanding them.
        addRequirements(shooter, turret);
        // Do not require drivetrain unless you will override driver control.
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;
        ChassisSpeeds robotRel = drivetrain.getState().Speeds;

        // Convert to field-relative speeds using current pose heading
        ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRel, pose.getRotation());

        Translation2d goalPos = turret.getGoalLocation();

        ShootingSolver.ShooterState state = null;
        double dist = 0.0;

        // Two-iteration "ghost pose" refinement
        for (int i = 0; i < 2; i++) {
            double ghostX = pose.getX() + fieldRel.vxMetersPerSecond * lastToF;
            double ghostY = pose.getY() + fieldRel.vyMetersPerSecond * lastToF;

            Translation2d ghost = new Translation2d(ghostX, ghostY);
            dist = ghost.getDistance(goalPos);

            state = ShootingSolver.solve(dist);
            lastToF = state.tof;
        }

        // Aim from ghost pose to goal
        double ghostX = pose.getX() + fieldRel.vxMetersPerSecond * lastToF;
        double ghostY = pose.getY() + fieldRel.vyMetersPerSecond * lastToF;
        Rotation2d aimFieldAngle = goalPos.minus(new Translation2d(ghostX, ghostY)).getAngle();

        // Apply to hardware
        shooter.setDifferentShooterRPMs(state.bottomRPM, state.topRPM);
        shooter.setHoodAngleDegrees(Math.toDegrees(state.angleRad));
        turret.aimFieldAngle(aimFieldAngle);
    }
}