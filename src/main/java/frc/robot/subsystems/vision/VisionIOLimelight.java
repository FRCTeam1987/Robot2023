package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
  public static VisionIOLimelight getInstance() {
    return instance;
  }

  private static VisionIOLimelight instance;
  public static int row = 0;
  List<VisionIOLimelightBase> limelights = new ArrayList<>();
  public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");

  public VisionIOLimelight(String... limelights) {
    instance = this;
    for (String name : limelights) {
      this.limelights.add(new VisionIOLimelightBase(name));
    }
    LIMELIGHT_TAB.addString("Best Bot Pose: ", this::getBestBotPoseStr);
  }

  public VisionIOLimelightBase getBestLimelight() {
    return limelights.stream().max(Comparator.comparing(VisionIOLimelightBase::getSeenTags)).get();
  }

  public Pose3d getBestBotPose() {
    return getBestLimelight().getBotPose();
  }

  public String getBestBotPoseStr() {
    try {
      return getBestBotPose().toString();
    } catch (Exception ignored) {
      return "No best bot pose";
    }
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    for (VisionIOLimelightBase limelight : limelights) {
      String name = limelight.limelightNameFormatted;
      try {
        inputs
            .getClass()
            .getField(name + "VisibleTags")
            .setLong(name + "VisibleTags", limelight.getSeenTags());
        inputs
            .getClass()
            .getField(name + "FrameMillis")
            .setLong(name + "FrameMillis", limelight.getFrameMillis());
        inputs.getClass().getField(name + "Json").set(name + "Json", limelight.getRawJson());
      } catch (IllegalAccessException | NoSuchFieldException e) {
        DriverStation.reportError("WARNING! Reflection broke in VisionIO.", true);
      }
    }
  }
}
