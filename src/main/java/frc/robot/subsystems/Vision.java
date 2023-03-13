package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Vision extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  public static int row = 0;
  static final List<Limelight> limelights = new ArrayList<>();

  public Vision(String... limelightsIn) {
    for (String name : limelightsIn) {
      limelights.add(new Limelight(name));
    }
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
  }

  public Limelight getBestLimelight() {
    try {
      return limelights.stream().max(Comparator.comparing(Limelight::getVisibleTagCount)).get();
    } catch (Exception e) {
      return limelights.get(0);
    }
  }

  @Override
  public void periodic() {

    Limelight limelight = getBestLimelight();
    try {
      double[] pose = limelight.getBotPose();
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(pose[0], pose[1], pose[2]),
              new Rotation3d(pose[3], pose[4], pose[5]));
      // poseEstimator.addVisionMeasurement(pose3d.toPose2d(), pose[6]);
    } catch (Exception ignored) {

    }
  }
}
