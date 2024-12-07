package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;

public class IntakeSubsystem extends SubsystemBase {    
    private final TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
    private double speed;
    
    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);
        intakeMotor.configContinuousCurrentLimit(30);
    }

    public void periodic() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setIntakeSpeed(double speed) {
        this.speed = speed;
    }

    public void runIntake() {
        setIntakeSpeed(SuperstructureConstants.INTAKE_SPEED);
    }


    public void stop() {
        speed = 0;
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public double getSpeed() {
        return speed;
    }
}