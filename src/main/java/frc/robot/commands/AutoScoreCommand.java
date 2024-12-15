package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.LimelightTargetTracking;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(SwerveSubsystem swerveSubsystem, PivotSubsystem pivotSubsystem,
            IntakeSubsystem intakeSubsystem, LimelightTargetTracking limelightTargetTracking) {
        addCommands(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(SuperstructureConstants.PARK_SET_POINT)),
                new InstantCommand(() -> swerveSubsystem.resetPose(new Pose2d())),
                new AutoAimCommand(swerveSubsystem, limelightTargetTracking, 5.0),
                new InstantCommand(() -> swerveSubsystem.resetPose(new Pose2d())),
                new WaitCommand(.5),
                new DriveToDistanceCommand(swerveSubsystem, 0, Units.inchesToMeters(12), 0, 1.0),
                new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(SuperstructureConstants.OUTTAKE_SPEED)),
                new WaitCommand(3),
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(SuperstructureConstants.SCORE_SET_POINT)));
    }
}