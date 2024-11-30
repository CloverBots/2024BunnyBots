package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.LimelightTargetTracking;

public class AutoAimCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private LimelightTargetTracking limelightTargetTracker;
    private double time;
    private Timer timer;

    public AutoAimCommand(SwerveSubsystem swerveSubsystem, LimelightTargetTracking limelightTargetTracker,
            double time) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightTargetTracker = limelightTargetTracker;
        this.time = time;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        swerveSubsystem.setRotationOverride(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getAlliance() == Alliance.Blue) {
            swerveSubsystem.setRotationTarget(Rotation2d.fromDegrees(limelightTargetTracker.getAngleToTote()));
        } else {
            swerveSubsystem.setRotationTarget(Rotation2d.fromDegrees(limelightTargetTracker.getAngleToTote())
                    .rotateBy(Rotation2d.fromDegrees(180)));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerveSubsystem.setRotationOverride(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (limelightTargetTracker.getAngleToTote() < 3 || timer.get() > time) {
            return true;
        } else {
            return false;
        }
    }
}