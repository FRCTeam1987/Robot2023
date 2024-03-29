package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  class VisionIOInputs implements LoggableInputs {
    public double[] botPoseLatency = new double[] {};

    public void toLog(LogTable table) {
      table.put("poseLatency", botPoseLatency);
    }

    public void fromLog(LogTable table) {
      botPoseLatency = table.getDoubleArray("poseLatency", new double[] {});
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
