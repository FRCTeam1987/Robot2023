package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface
VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public String[] json = new String[] {};

    public void toLog(LogTable table) {
      table.put("json", json);
    }

    public void fromLog(LogTable table) {
      json = table.getStringArray("json", new String[] {});
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
