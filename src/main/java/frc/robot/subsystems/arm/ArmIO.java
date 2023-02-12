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
}
