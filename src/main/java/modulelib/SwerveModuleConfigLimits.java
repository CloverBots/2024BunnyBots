package modulelib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;

/**
 * Represents a swerve module implemented using CTR Electronics Phoenix V6 API.
 * It uses TalonFX, i.e. Falcon500 or Kraken60, motors and CANCoders for control.
 */
public class SwerveModuleConfigLimits {
    // Hardware objects
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    // Status signals used for synchronizing odometry.
    private final StatusSignal<Double> drivePositionStatusSignal;
    private final StatusSignal<Double> driveVelocityStatusSignal;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTemperature;

    private final StatusSignal<Double> steerPositionStatusSignal;
    private final StatusSignal<Double> steerVelocityStatusSignal;
    private final StatusSignal<Double> steerAppliedVolts;
    private final StatusSignal<Double> steerSupplyCurrent;
    private final StatusSignal<Double> steerTemperature;
    private final StatusSignal<Double> steerEncoderPositionStatusSignal;
    private final StatusSignal<Double> steerEncoderAbsolutePosition;

    // Control mode setters
    private final VoltageOut driveControlSetter;
    private final PositionVoltage steerControlSetter;
    private final MotionMagicVoltage motionMagicSetter;

    // variables
    protected final double driveRotationsPerMeter;

    private final double maxVelocityMetersPerSecond;

    protected SwerveModuleConfig swerveModuleConfig;

    // Target Variables. Used only for data logging
    protected double targetVelocityMetersPerSeconds = 0.0;
    protected double targetSteerAngleRadians = 0.0;

    private final double VELOCITY_COEFFICIENT = 1.10;

    /**
     * Initializes the motors, encoder, and the settings for each of the devices.
     *
     * @param swerveModuleConfig The configuration for the swerve module.
     * @param supportsPro        Whether the motor controller supports Pro mode.
     */
    public SwerveModuleConfigLimits(SwerveModuleConfig swerveModuleConfig) {
        this.swerveModuleConfig = swerveModuleConfig;

        driveMotor = new TalonFX(swerveModuleConfig.driveMotorId);
        steerMotor = new TalonFX(swerveModuleConfig.angleMotorId);
        steerEncoder = new CANcoder(
                swerveModuleConfig.encoderId);

        // Configure Drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0 = new Slot0Configs();

        driveConfig.CurrentLimits.SupplyCurrentLimit = 50;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveConfig.CurrentLimits.StatorCurrentLimit = 200;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        driveConfig.MotorOutput.Inverted = swerveModuleConfig.drive_inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure Steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0 = new Slot0Configs();

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / DriveConstants.ANGLE_GEAR_RATIO;
        steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * DriveConstants.ANGLE_GEAR_RATIO;
        steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        steerConfig.Slot1.kP = 16;
        steerConfig.Slot1.kI = 0;
        steerConfig.Slot1.kD = 0;
        steerConfig.Slot1.kS = 0.8;
        steerConfig.Slot1.kV = 0.1224;

        // CANcoder Configuration, apply offset.
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();

        var response = steerEncoder.getConfigurator().apply(cancoderConfigs);

        if (!response.isOK()) {
            System.out.println("CANcoder ID " + swerveModuleConfig.encoderId
                    + " failed config with error " + response.toString());
        }

        // Modify configuration to use remote CANcoder fused
        steerConfig.Feedback.FeedbackRemoteSensorID = swerveModuleConfig.encoderId;

        steerConfig.Feedback.RotorToSensorRatio = DriveConstants.ANGLE_GEAR_RATIO;

        steerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = 80;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Get control modes for drive and steer motors
        driveControlSetter = new VoltageOut(0.0).withUpdateFreqHz(0);
        steerControlSetter = new PositionVoltage(0.0).withSlot(0).withUpdateFreqHz(0);
        motionMagicSetter = new MotionMagicVoltage(.00).withSlot(1).withUpdateFreqHz(0);

        // Get signals and set update rate 100hz signals
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();
        steerAppliedVolts = steerMotor.getMotorVoltage();
        steerSupplyCurrent = steerMotor.getSupplyCurrent();
        steerTemperature = steerMotor.getDeviceTemp();
        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveAppliedVolts,
                driveSupplyCurrent,
                steerAppliedVolts,
                steerSupplyCurrent,
                driveTemperature,
                steerTemperature,
                steerEncoderAbsolutePosition);

        // Set the status signals to support synchronized odometry.
        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();
        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                drivePositionStatusSignal,
                driveVelocityStatusSignal,
                steerPositionStatusSignal,
                steerVelocityStatusSignal,
                steerEncoderPositionStatusSignal);

        BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotor.getFaultField(), steerMotor.getFaultField());

        /* Calculate the ratio of drive motor rotation to meter on ground */
        var rotationsPerWheelRotation = DriveConstants.DRIVE_GEAR_RATIO;
        var metersPerWheelRotation = 2 * Math.PI * DriveConstants.WHEEL_RADIUS;
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        maxVelocityMetersPerSecond = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

        // When optimizing bus utilization, make sure all Signals in use have their update frequency set.
        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    public void setModuleState(SwerveModuleState state, boolean steerMotionMagicEnabled) {
        double angleToSetRotations = state.angle.getRotations();
        if (steerMotionMagicEnabled) {
            steerMotor.setControl(motionMagicSetter.withPosition(angleToSetRotations));
        } else {
            steerMotor.setControl(steerControlSetter.withPosition(angleToSetRotations));
        }

        double velocityToSet = state.speedMetersPerSecond;

        var voltage = (velocityToSet / maxVelocityMetersPerSecond) * 12.0;

        if (DriverStation.isAutonomous()) {
            voltage *= VELOCITY_COEFFICIENT;
        }

        driveMotor.setControl(driveControlSetter.withOutput(voltage));

        // Make these values available for logging
        targetSteerAngleRadians = Units.rotationsToRadians(angleToSetRotations);
        targetVelocityMetersPerSeconds = velocityToSet;
    }
}