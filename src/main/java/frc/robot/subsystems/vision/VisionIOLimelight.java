package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
  public static VisionIOLimelight getInstance() {
    return instance;
  }

  private static VisionIOLimelight instance;
  public static int row = 0;
  static final List<VisionIOLimelightBase> limelights = new ArrayList<>();

  public VisionIOLimelight(String... limelightsIn) {
    instance = this;
    for (String name : limelightsIn) {
      limelights.add(new VisionIOLimelightBase(name));
    }
  }

  public VisionIOLimelightBase getBestLimelight() {
    try {
      return limelights.stream()
          .sorted(Comparator.comparing(VisionIOLimelightBase::getTargetArea))
          .max(Comparator.comparing(VisionIOLimelightBase::getVisibleTagCount))
          .get();
    } catch (Exception e) {
      return limelights.get(0);
    }
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    int size = limelights.size();
    inputs.botPoseLatency = new double[size * 7];
    int i2 = 0;
    for (VisionIOLimelightBase limelight : limelights) {
      double[] poseLatency = limelight.getBotPose();
      inputs.botPoseLatency[i2++] = poseLatency[0];
      inputs.botPoseLatency[i2++] = poseLatency[1];
      inputs.botPoseLatency[i2++] = poseLatency[2];
      inputs.botPoseLatency[i2++] = poseLatency[3];
      inputs.botPoseLatency[i2++] = poseLatency[4];
      inputs.botPoseLatency[i2++] = poseLatency[5];
      inputs.botPoseLatency[i2++] = poseLatency[6];
    }
  }
}
