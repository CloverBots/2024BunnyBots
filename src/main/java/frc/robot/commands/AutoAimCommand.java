package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
        swerveSubsystem.rotationOverride = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSubsystem.defaultDrive(0, 0.05, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if ((limelightTargetTracker.getAngleToTote() <= 1 && limelightTargetTracker.getAngleToTote() >= -1) || timer.get() > time) {
            return true;
        } else {
            return false;
        }
    }
}