package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public static String frontLeftJson = "",
        frontRightJson = "",
        backLeftJson = "",
        backRightJson = "";
    public static long frontLeftVisibleTags = 0L,
        frontRightVisibleTags = 0L,
        backLeftVisibleTags = 0L,
        backRightVisibleTags = 0L;
    public static long frontLeftFrameMillis = 0L,
        frontRightFrameMillis = 0L,
        backLeftFrameMillis = 0L,
        backRightFrameMillis = 0L;

    public void toLog(LogTable table) {
      table.put("frontLeftJson", frontLeftJson);
      table.put("frontRightJson", frontRightJson);
      table.put("backLeftJson", backLeftJson);
      table.put("backRightJson", backRightJson);

      table.put("frontLeftVisibleTags", frontLeftVisibleTags);
      table.put("frontRightVisibleTags", frontRightVisibleTags);
      table.put("backLeftVisibleTags", backLeftVisibleTags);
      table.put("backRightVisibleTags", backRightVisibleTags);

      table.put("frontLeftFrameMillis", frontLeftFrameMillis);
      table.put("frontRightFrameMillis", frontRightFrameMillis);
      table.put("backLeftFrameMillis", backLeftFrameMillis);
      table.put("backRightFrameMillis", backRightFrameMillis);
    }

    public void fromLog(LogTable table) {
      frontLeftJson = table.getString("frontLeftJson", null);
      frontRightJson = table.getString("frontRightJson", null);
      backLeftJson = table.getString("backLeftJson", null);
      backRightJson = table.getString("backRightJson", null);

      frontLeftVisibleTags = table.getInteger("frontLeftVisibleTags", 0L);
      frontRightVisibleTags = table.getInteger("frontRightVisibleTags", 0L);
      backLeftVisibleTags = table.getInteger("backLeftVisibleTags", 0L);
      backRightVisibleTags = table.getInteger("backRightVisibleTags", 0L);

      frontLeftFrameMillis = table.getInteger("frontLeftFrameMillis", 0L);
      frontRightFrameMillis = table.getInteger("frontRightFrameMillis", 0L);
      backLeftFrameMillis = table.getInteger("backLeftFrameMillis", 0L);
      backRightFrameMillis = table.getInteger("backRightFrameMillis", 0L);
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
