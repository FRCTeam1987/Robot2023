package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ArmIOInputs inputs) {}
}
