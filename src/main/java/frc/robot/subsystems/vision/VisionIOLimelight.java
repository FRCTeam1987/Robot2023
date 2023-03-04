package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team6328.util.Alert;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionIOLimelight implements VisionIO {

  public static final Alert limelightAlert =
      new Alert("A limelight was disconnected / has bugged out!", Alert.AlertType.ERROR);

  public static int row = 0;
  static List<VisionIOLimelightBase> limelights = new ArrayList<>();

  public VisionIOLimelight(String... limelightsIn) {
    for (String name : limelightsIn) {
      limelights.add(new VisionIOLimelightBase(name));
    }
  }

  @Override
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
  public void updateAlliance() {
    for (VisionIOLimelightBase limelight : limelights) {
      limelight.setAlliance(DriverStation.getAlliance() == DriverStation.Alliance.Blue);
    }
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    int size = limelights.size();
    inputs.botPoseLatency = new double[size * 7];
    int totalIndex = 0;
    try {
      for (VisionIOLimelightBase limelight : limelights) {
        double[] poseLatency = limelight.getBotPose();
        for (int i = 0; i < 7; i++) {
          inputs.botPoseLatency[totalIndex++] = poseLatency[i];
        }
      }
    } catch (Exception ignored) {
      limelightAlert.set(true);
    }
  }
}
