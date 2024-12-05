package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;

public class BlowerSubsystem extends SubsystemBase {    
    private final TalonSRX blowerMotor = new TalonSRX(Constants.BLOWER_MOTOR_ID);
    private double speed;
    
    public BlowerSubsystem() {
        blowerMotor.setNeutralMode(NeutralMode.Coast);
        blowerMotor.configContinuousCurrentLimit(30);
    }

    public void periodic() {
        blowerMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void runBlower() {
        this.speed = SuperstructureConstants.BLOWER_SPEED;
    }

    public void stop() {
        speed = 0;
        blowerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}