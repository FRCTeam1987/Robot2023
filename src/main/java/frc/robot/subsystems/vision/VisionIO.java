package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    //public String[] json = new String[]{};
    public long[] frameTimes = new long[]{};
    public double[] canSeeTag = new double[]{};

    public void toLog(LogTable table) {
      //table.put("json", json);

      table.put("frameTimes", frameTimes);
      table.put("canSeeTag", canSeeTag);
    }

    public void fromLog(LogTable table) {
      //json = table.getStringArray("json", new String[]{});
      frameTimes = table.getIntegerArray("frameTimes", new long[]{});
      canSeeTag = table.getDoubleArray("canSeeTag", new double[]{});
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
