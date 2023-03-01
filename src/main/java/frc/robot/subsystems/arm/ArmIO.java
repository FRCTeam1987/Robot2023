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

  public default double getArmLength() {
    return 0.0;
  }

  public default void setArmLength(double length) {}

  public default double getArmAngle() {
    return 0.0;
  }

  public default void setArmAngle(double angle) {}
}
