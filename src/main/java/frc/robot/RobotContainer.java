package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SuperstructureCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TimeOfFlightSubsystem;
import limelight.LimelightTargetTracking;

public class RobotContainer {
    public final LimelightTargetTracking visionTargetTracker = new LimelightTargetTracking(
            VisonConstants.visionConfiguration);
    private Pigeon2 gyro = new Pigeon2(Constants.GYRO_ID);
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(gyro);
    private final TimeOfFlightSubsystem timeOfFlightSubsystem = new TimeOfFlightSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

    private final XboxController driverController = new XboxController(Constants.CONTROLLER_DRIVE_PORT);
    private final XboxController operatorController = new XboxController(Constants.CONTROLLER_OPERATOR_PORT);

    private final SuperstructureCommand superstructureCommand = new SuperstructureCommand(timeOfFlightSubsystem,
            pivotSubsystem, intakeSubsystem,
            driverController::getRightTriggerAxis,
            driverController::getLeftBumper,
            operatorController::getBackButton,
            driverController::getRightBumper,
            operatorController::getLeftY);

    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        configureAutoCommands();

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(
                new DriveCommand(
                        swerveSubsystem,
                        () -> getScaledXY(),
                        () -> scaleRotationAxis(driverController.getRightX())));

        configureBindings();
    }

    // Will run once any time the robot is enabled, in any mode (Doesn't matter if
    // Teleop / Autonomous)
    public void onEnable() {
        resetGyro();
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public void teleopInit() {
        intakeSubsystem.setDefaultCommand(superstructureCommand);
    }

    public void teleopPeriodic() {
        timeOfFlightSubsystem.isBallonLoaded();
        if (driverController.getStartButton() == true) {
            resetGyro();
        }
    }

    public void onAutonomousEnable() {
        resetGyro();
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Will run once any time the robot is disabled. */
    public void disabledInit() {}

    private void configureBindings() {}

    private void configureAutoCommands() {
        NamedCommands.registerCommand("Score", new AutoScoreCommand(swerveSubsystem, pivotSubsystem, intakeSubsystem, visionTargetTracker));
    }

    private double squared(double input) {
        return Math.copySign(input * input, input);
    }

    private double scaleRotationAxis(double input) {
        double rotation = -deadband(squared(input), DriveConstants.deadband) * swerveSubsystem.getMaxAngleVelocity(); 
        if (driverController.getLeftTriggerAxis() > 0.5) {
            rotation = rotation * Constants.DriveConstants.TELEOP_SLOW_ANGULAR_SCALE_FACTOR;
        } else {
            rotation = rotation * Constants.DriveConstants.TELEOP_NORMAL_ANGULAR_SCALE_FACTOR;
        } 
        return rotation;
    }

    private double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        } else {
            return input;
        }
    }

    private double[] getXY() {
        double[] xy = new double[2];
        xy[0] = deadband(driverController.getLeftX(), DriveConstants.deadband);
        xy[1] = deadband(driverController.getLeftY(), DriveConstants.deadband);
        return xy;
    }

    private double[] getScaledXY() {
        double[] xy = getXY();

        // Convert to Polar coordinates
        double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
        double theta = Math.atan2(xy[1], xy[0]);

        // Square radius and scale by max velocity
        r = r * r * swerveSubsystem.getMaxVelocity();

        // Convert to Cartesian coordinates
        xy[0] = r * Math.cos(theta);
        xy[1] = r * Math.sin(theta);

        if (driverController.getLeftTriggerAxis() > 0.5) {
            xy[0] = xy[0] / 25;
            xy[1] = xy[1] / 25;
        }

        return xy;
    }
}