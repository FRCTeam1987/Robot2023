package frc.robot.subsystems;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;
import static frc.robot.Constants.TAB_VISION;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve3061.RobotOdometry;
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
      // TODO: Uncomment me for vision pose updates
      /*      double[] pose = limelight.getBotPose();
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(pose[0], pose[1], pose[2]),
              new Rotation3d(pose[3], pose[4], pose[5]));
      poseEstimator.addVisionMeasurement(pose3d.toPose2d(), pose[6]);*/
    } catch (Exception ignored) {

    }
  }

  private static class Limelight {
    public final String limelightName;
    private final DoubleArraySubscriber botPoseSubscriber;
    private final StringSubscriber jsonSubscriber;

    public Limelight(String limelightName) {
      this.limelightName = limelightName;

      String limelightNameShort = limelightName.replace("limelight-", "");
      NetworkTable inst = NetworkTableInstance.getDefault().getTable(limelightName);
      // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
      inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
      TAB_VISION
          .addNumber(limelightNameShort + " count", this::getVisibleTagCount)
          .withPosition(0, Vision.row++);
      botPoseSubscriber = inst.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
      jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
    }

    public String getRawJson() {
      return jsonSubscriber.get();
    }

    /**
     * This is cheeky and may break in the future. Each fiducial object in the json array contains
     * one instance of the letter "m". We parse through the entire json string and grab the count of
     * the letter m.
     */
    public int getVisibleTagCount() {
      return (int) jsonSubscriber.get().codePoints().filter(ch -> ch == 'm').count();
    }

    public double[] getBotPose() {
      return botPoseSubscriber.get();
    }
  }
}
