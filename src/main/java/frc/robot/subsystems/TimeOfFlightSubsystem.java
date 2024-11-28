package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase {

    TimeOfFlight distanceSensor = new TimeOfFlight(0);

    public static final double BALLON_LOADED_DISTANCE = 160;

    public TimeOfFlightSubsystem() {
        distanceSensor.setRangingMode(RangingMode.Short, 24);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ToF", distanceSensor.getRange());
    }

    public boolean isBallonLoaded() {
        if (distanceSensor.getRange() < BALLON_LOADED_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }
}