package frc.robot.subsystems.vision;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.sql.Driver;

public class VisionIOLimelightBase {
  private final NetworkTable inst;
  public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
  public final String limelightName;
  private StringSubscriber jsonSubscriber;
  private DoubleSubscriber targetSubscriber;
  private DoubleSubscriber latencySubscriber;
  public VisionIOLimelightBase(String limelightName) {
    int column = 0;
    this.limelightName = limelightName;

    String limelightNameShort = limelightName.replace("limelight-", "");
    inst = NetworkTableInstance.getDefault().getTable(limelightName);
    // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    LIMELIGHT_TAB
        .addDouble(limelightNameShort + " Target Visible", this::canSeeTarget)
        .withPosition(column++, VisionIOLimelight.row);
    LIMELIGHT_TAB
        .addString(limelightNameShort + " pose", this::getBotPoseStr)
        .withPosition(column++, VisionIOLimelight.row);
    VisionIOLimelight.row++;
    //jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
    targetSubscriber = inst.getDoubleTopic("tv").subscribe(0.0);
    latencySubscriber = inst.getDoubleTopic("tl").subscribe(-1.0);
  }

  /*public String getRawJson() {
    return jsonSubscriber.get();
  }*/

  public Pose3d getBotPose() {
    try {
      double[] pose = inst.getEntry("botpose_wpiblue").getDoubleArray(new double[]{});
      return new Pose3d(
          new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
    } catch (Exception ignored) {
      // This throws an exception when the bot doesn't have a pose. Just ignore it and move on,
      // accepting that we currently don't have a pose.
      return null;
    }
  }

  public String getBotPoseStr() {
    try {
      return getBotPose().toString();
    } catch (Exception e) {
      return "No pose detected";
    }
  }

  public long getFrameMillis() {
    return (long) (System.currentTimeMillis() - latencySubscriber.get());
  }

  public double canSeeTarget() {
    return targetSubscriber.get();
  }
}
