package frc.robot.subsystems.vision;

import frc.lib.team6328.util.Alert;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
  public static VisionIOLimelight getInstance() {
    return instance;
  }

  private static VisionIOLimelight instance;
  public static int row = 0;
  static List<VisionIOLimelightBase> limelights = new ArrayList<>();

  public static final Alert limelightAlert =
      new Alert("A limelight was disconnected / has bugged out!", Alert.AlertType.ERROR);

  public VisionIOLimelight(String... limelightsIn) {
    instance = this;
    for (String name : limelightsIn) {
      limelights.add(new VisionIOLimelightBase(name));
    }
  }

  public VisionIOLimelightBase getBestLimelight() {
    try {
      return limelights.stream()
          .max(Comparator.comparing(VisionIOLimelightBase::getVisibleTagCount))
          .get();
    } catch (Exception e) {
      return limelights.get(0);
    }
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    try {
      int size = limelights.size();
      inputs.json = new String[size];
      for (int i = 0; i < size; i++) {
        inputs.json[i] = limelights.get(i).getRawJson();
      }
    } catch (Exception ignored) {
      limelightAlert.set(true);
    }
  }
}
