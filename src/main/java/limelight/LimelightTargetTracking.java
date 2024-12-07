package limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightTargetTracking {

    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    private static final String LIMELIGHT_TABLE_ENTRY_X = "tx";
    private static final String LIMELIGHT_TABLE_ENTRY_Y = "ty";

    // tv = 0 if no valid targets identified, tv = 1 for valid target
    private static final String LIMELIGHT_TABLE_ENTRY_VALID = "tv";

    private final LimelightConfiguration configuration;
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    public final NetworkTableEntry tv;

    public LimelightTargetTracking(LimelightConfiguration configuration) {
        this.configuration = configuration;

        table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);
        tx = table.getEntry(LIMELIGHT_TABLE_ENTRY_X);
        ty = table.getEntry(LIMELIGHT_TABLE_ENTRY_Y);
        tv = table.getEntry(LIMELIGHT_TABLE_ENTRY_VALID);
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    /**
     * Gets the center Y offset of the vision target in degrees
     * ranging from -20.5 to 20.5.
     */
    public double getTy() {
        return ty.getDouble(0.0);
    }

    /**
     * Get whether there is a valid target visible
     */
    public boolean isValid() {
        if (tv.getNumber(0).intValue() == 0) {
            return false;
        }
        return true;
    }

    /**
     * Computes the distance to the target, usually in inches.
     * 
     * @param config The configuration describing camera and field measurements.
     */
    public double computeTargetDistance() {
        if (isValid()) {
            var targetHeight = configuration.getTargetHeight();
            var cameraHeight = configuration.getCameraHeight();
            var cameraPitch = configuration.getCameraPitch();

            // convert degrees to radians because Math.tan uses radians
            return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraPitch + getTy()));
        } else {
            return -1;
        }
    }

    public double getAngleToTote() {
        return getTx();
      }
}