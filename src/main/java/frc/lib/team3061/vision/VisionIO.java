package frc.lib.team3061.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public static String limelightFrontLeftJson = "", limelightFrontRightJson = "", limelightRearLeftJson = "", limelightRearRightJson = "";
    public static long limelightFrontLeftVisibleTags = 0L, limelightFrontRightVisibleTags = 0L, limelightRearLeftVisibleTags = 0L, limelightRearRightVisibleTags = 0L;
    public static long limelightFrontLeftFrameMillis = 0L, limelightFrontRightFrameMillis = 0L, limelightRearLeftFrameMillis = 0L, limelightRearRightFrameMillis = 0L;

    public void toLog(LogTable table) {
      table.put("limelightFrontLeftJson", limelightFrontLeftJson);
      table.put("limelightFrontRightJson", limelightFrontRightJson);
      table.put("limelightRearLeftJson", limelightRearLeftJson);
      table.put("limelightRearRightJson", limelightRearRightJson);

      table.put("limelightFrontLeftVisibleTags", limelightFrontLeftVisibleTags);
      table.put("limelightFrontRightVisibleTags", limelightFrontRightVisibleTags);
      table.put("limelightRearLeftVisibleTags", limelightRearLeftVisibleTags);
      table.put("limelightRearRightVisibleTags", limelightRearRightVisibleTags);

      table.put("limelightFrontLeftFrameMillis", limelightFrontLeftFrameMillis);
      table.put("limelightFrontRightFrameMillis", limelightFrontRightFrameMillis);
      table.put("limelightRearLeftFrameMillis", limelightRearLeftFrameMillis);
      table.put("limelightRearRightFrameMillis", limelightRearRightFrameMillis);
    }

    public void fromLog(LogTable table) {
      limelightFrontLeftJson = table.getString("limelightFrontLeftJson", null);
      limelightFrontRightJson = table.getString("limelightFrontRightJson", null);
      limelightRearLeftJson = table.getString("limelightRearLeftJson", null);
      limelightRearRightJson = table.getString("limelightRearRightJson", null);

      limelightFrontLeftVisibleTags = table.getInteger("limelightFrontLeftVisibleTags", 0L);
      limelightFrontRightVisibleTags = table.getInteger("limelightFrontRightVisibleTags", 0L);
      limelightRearLeftVisibleTags = table.getInteger("limelightRearLeftVisibleTags", 0L);
      limelightRearRightVisibleTags = table.getInteger("limelightRearRightVisibleTags", 0L);

      limelightFrontLeftFrameMillis = table.getInteger("limelightFrontLeftFrameMillis", 0L);
      limelightFrontRightFrameMillis = table.getInteger("limelightFrontRightFrameMillis", 0L);
      limelightRearLeftFrameMillis = table.getInteger("limelightRearLeftFrameMillis", 0L);
      limelightRearRightFrameMillis = table.getInteger("limelightRearRightFrameMillis", 0L);
    }

  }
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
