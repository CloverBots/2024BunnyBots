package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;

public class PivotSubsystem extends PIDSubsystem {
    private final CANSparkMax pivotMotor;
    double speed = 0;

    public PivotSubsystem() {
        super(new PIDController(0.02, 0.0045, 0));
        getController().setTolerance(0.5);
        getController().enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.
        disable(); // start with PID disabled

        pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR, MotorType.kBrushless);

        pivotMotor.setInverted(true);

        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor.setSmartCurrentLimit(10);
    }

    @Override
    public double getMeasurement() {
        return getPivotPosition();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        output = MathUtil.clamp(output, -0.1, 0.1);
        speed = output;
        if (pivotMotor != null) {
            pivotMotor.set(speed);
        }
    }

    public void resetPivotEncoder() {
        pivotMotor.getEncoder().setPosition(getPivotPosition());
    }

    public void setSpeed(double speed) {
        disable();
        pivotMotor.set(speed);
        this.speed = speed;
    }

    public void stop() {
        pivotMotor.set(0);
        disable();
    }

    public void setPivotPosition(double position) {
        getController().reset();
        setSetpoint(position);
        enable();
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public boolean pivotReady() {
        if (Math.abs(getPivotPosition() - getSetpoint()) < 2) {
            return true;
        } else {
            return false;
        }
    }

    public double getPivotPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {

        super.periodic(); // super controlls calling PID stuff

        if ((getPivotPosition() < SuperstructureConstants.LOWER_ENDPOINT &&
                speed < 0) || (getPivotPosition() > SuperstructureConstants.UPPER_ENDPOINT && speed > 0)) {
            System.out.println("PIVOT ENDPOINT reached!!!");
            speed = 0;
            disable();
        }

        SmartDashboard.putNumber("Pivot Encoder", getPivotPosition());

        pivotMotor.set(speed);
    }
}