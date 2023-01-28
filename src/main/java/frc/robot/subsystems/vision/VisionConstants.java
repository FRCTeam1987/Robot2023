package frc.robot.subsystems.vision;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // FIXME: add the current year's AprilTag field layout file to the deploy directory and update
  // this constant with the current year's AprilTag field layout file

  public static final double MAX_POSE_DIFFERENCE_METERS = 1.0;
}
