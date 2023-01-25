package frc.lib.team3061.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public static String frontLeftJson = "", frontRightJson = "", rearLeftJson = "", rearRightJson = "";
    public static long frontLeftVisibleTags = 0L, frontRightVisibleTags = 0L, rearLeftVisibleTags = 0L, rearRightVisibleTags = 0L;
    public static long frontLeftFrameMillis = 0L, frontRightFrameMillis = 0L, rearLeftFrameMillis = 0L, rearRightFrameMillis = 0L;

    public void toLog(LogTable table) {
      table.put("frontLeftJson", frontLeftJson);
      table.put("frontRightJson", frontRightJson);
      table.put("rearLeftJson", rearLeftJson);
      table.put("rearRightJson", rearRightJson);

      table.put("frontLeftVisibleTags", frontLeftVisibleTags);
      table.put("frontRightVisibleTags", frontRightVisibleTags);
      table.put("rearLeftVisibleTags", rearLeftVisibleTags);
      table.put("rearRightVisibleTags", rearRightVisibleTags);

      table.put("frontLeftFrameMillis", frontLeftFrameMillis);
      table.put("frontRightFrameMillis", frontRightFrameMillis);
      table.put("rearLeftFrameMillis", rearLeftFrameMillis);
      table.put("rearRightFrameMillis", rearRightFrameMillis);
    }

    public void fromLog(LogTable table) {
      frontLeftJson = table.getString("frontLeftJson", null);
      frontRightJson = table.getString("frontRightJson", null);
      rearLeftJson = table.getString("rearLeftJson", null);
      rearRightJson = table.getString("rearRightJson", null);

      frontLeftVisibleTags = table.getInteger("frontLeftVisibleTags", 0L);
      frontRightVisibleTags = table.getInteger("frontRightVisibleTags", 0L);
      rearLeftVisibleTags = table.getInteger("rearLeftVisibleTags", 0L);
      rearRightVisibleTags = table.getInteger("rearRightVisibleTags", 0L);

      frontLeftFrameMillis = table.getInteger("frontLeftFrameMillis", 0L);
      frontRightFrameMillis = table.getInteger("frontRightFrameMillis", 0L);
      rearLeftFrameMillis = table.getInteger("rearLeftFrameMillis", 0L);
      rearRightFrameMillis = table.getInteger("rearRightFrameMillis", 0L);
    }

  }
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
