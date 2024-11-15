package modulelib;

/** Add your docs here. */
public class SwerveModuleConfig {
  public final int driveMotorId;
  public final int angleMotorId;
  public final int encoderId;
  public final boolean drive_inverted;

  public SwerveModuleConfig(int driveMotorId, int angleMotorId, int encoderId, boolean drive_inverted) {
    this.driveMotorId = driveMotorId;
    this.angleMotorId = angleMotorId;
    this.encoderId = encoderId;
    this.drive_inverted = drive_inverted;
  }
}