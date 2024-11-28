package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.LimelightTargetTracking;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(SwerveSubsystem swerveSubsystem, PivotSubsystem pivotSubsystem,
            IntakeSubsystem intakeSubsystem, LimelightTargetTracking limelightTargetTracking) {
        addCommands(
                new InstantCommand(() -> swerveSubsystem.resetPose(new Pose2d())),
                new AutoAimCommand(swerveSubsystem, limelightTargetTracking, pivotSubsystem, 1.0),
                new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(72), 0, 0, 3.2)
                        .alongWith(
                                new InstantCommand(() -> pivotSubsystem
                                        .setPivotPosition(SuperstructureConstants.SCORE_SET_POINT))),
                new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(-SuperstructureConstants.INTAKE_SPEED)));
    }
}