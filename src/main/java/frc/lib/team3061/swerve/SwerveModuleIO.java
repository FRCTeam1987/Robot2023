package frc.lib.team3061.swerve;

import org.littletonrobotics.junction.AutoLog;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {
  @AutoLog
  class SwerveModuleIOInputs {
    double drivePositionDeg = 0.0;
    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveAppliedPercentage = 0.0;
    double driveCurrentAmps = 0.0;
    // double[] driveTempCelsius = new double[] {};

    double angleAbsolutePositionDeg = 0.0;
    double anglePositionDeg = 0.0;
    double angleVelocityRevPerMin = 0.0;
    double angleAppliedPercentage = 0.0;
    double angleCurrentAmps = 0.0;
    // double[] angleTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  default void setDriveMotorPercentage(double percentage) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  default void setAnglePosition(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setAngleBrakeMode(boolean enable) {}
}
