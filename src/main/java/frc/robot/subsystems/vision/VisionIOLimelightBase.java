package frc.robot.subsystems.vision;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionIOLimelightBase {
  public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
  public final String limelightName;
  private final DoubleArraySubscriber botPoseSubscriber;
  private final StringSubscriber jsonSubscriber;
  private final DoubleSubscriber targetSubscriber;
  private final DoubleSubscriber latencySubscriber;

  public VisionIOLimelightBase(String limelightName) {
    int column = 0;
    this.limelightName = limelightName;

    String limelightNameShort = limelightName.replace("limelight-", "");
    NetworkTable inst = NetworkTableInstance.getDefault().getTable(limelightName);
    // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    LIMELIGHT_TAB
        .addDouble(limelightNameShort + " Target Visible", this::canSeeTarget)
        .withPosition(column++, VisionIOLimelight.row);
    LIMELIGHT_TAB
        .addNumber(limelightNameShort + " count", this::getVisibleTagCount)
        .withPosition(column++, VisionIOLimelight.row);
    VisionIOLimelight.row++;
    botPoseSubscriber = inst.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
    targetSubscriber = inst.getDoubleTopic("tv").subscribe(0.0);
    latencySubscriber = inst.getDoubleTopic("tl").subscribe(-1.0);
  }

  public String getRawJson() {
    return jsonSubscriber.get();
  }

  /**
   * This is cheeky and may break in the future. Each fiducial object in the json array contains one
   * instance of the letter "m". We parse through the entire json string and grab the count of the
   * letter m.
   */
  public int getVisibleTagCount() {
    return (int) jsonSubscriber.get().codePoints().filter(ch -> ch == 'm').count();
  }

  public Pose3d getBotPose() {
    try {
      double[] pose = botPoseSubscriber.get();
      return new Pose3d(
          new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
    } catch (Exception ignored) {
      // This throws an exception when the bot doesn't have a pose. Just ignore it and move on,
      // accepting that we currently don't have a pose.
      return null;
    }
  }

  public long getFrameMillis() {
    return (long) (System.currentTimeMillis() - latencySubscriber.get());
  }

  public double canSeeTarget() {
    return targetSubscriber.get();
  }
}
