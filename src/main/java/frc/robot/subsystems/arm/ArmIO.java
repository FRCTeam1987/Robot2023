package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double[] currentAmps = new double[] {};
    public double[] currentVolts = new double[] {};
    public double armAbsoluteAngle;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default double getEncoderPosition() {
    return 0.0;
  }

  public default double getEncoderPositionNoOffset() {
    return 0.0;
  }

  public default double getTalonPosition() {
    return 0.0;
  }

  public default void rotateArmToAngle(double angle) {}

}
