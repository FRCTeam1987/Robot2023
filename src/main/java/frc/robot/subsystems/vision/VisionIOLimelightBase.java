package frc.robot.subsystems.vision;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionIOLimelightBase {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
  public static final ObjectMapper OM = new ObjectMapper();
  public final String limelightName;
  public String limelightNameFormatted = null;

  public VisionIOLimelightBase(String limelightName) {
    int column = 0;
    this.limelightName = limelightName;
    switch (limelightName) {
      case "limelight-fl":
        limelightNameFormatted = "frontLeft";
        break;
      case "limelight-fr":
        limelightNameFormatted = "frontRight";
        break;
      case "limelight-bl":
        limelightNameFormatted = "backLeft";
        break;
      case "limelight-br":
        limelightNameFormatted = "backRight";
        break;
    }
    String limelightNameShort = limelightName.replace("limelight-", "");

    // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    inst.getTable(limelightName).getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    LIMELIGHT_TAB
        .addNumber(limelightNameShort + " Y Axis", this::getYAxis)
        .withPosition(column++, VisionIOLimelight.row);
    LIMELIGHT_TAB
        .addBoolean(limelightNameShort + " Target Visible", this::canSeeTarget)
        .withPosition(column++, VisionIOLimelight.row);
    LIMELIGHT_TAB
        .addNumber(limelightNameShort + " Targets Seen", this::getSeenTags)
        .withPosition(column++, VisionIOLimelight.row);
    LIMELIGHT_TAB
        .addString(limelightNameShort + " pose", this::getBotPoseStr)
        .withPosition(column++, VisionIOLimelight.row);
    VisionIOLimelight.row++;
  }

  public double getYAxis() {
    return inst.getTable(limelightName).getEntry("ty").getDouble(0);
  }

  public JsonNode getJsonNode() {
    try {
      return OM.readTree(getRawJson());
    } catch (Exception ignored) {
      /*This exception is to catch null JSONs, which happens a fair amount. Maybe something to ask Limelight devs. */
    }
    return null;
  }

  public String getRawJson() {
    return inst.getTable(limelightName).getEntry("json").getString(null);
  }

  public Pose3d getBotPose() {
    try {
      double[] pose =
          inst.getTable(limelightName)
              .getEntry("botpose")
              .getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
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

  public int getSeenTags() {
    try {
      return getJsonNode().at("/Results/Fiducial").size();
    } catch (Exception ignored) {
      return 0;
    }
  }

  public long getFrameMillis() {
    return System.currentTimeMillis() - inst.getTable(limelightName).getEntry("tl").getInteger(-1L);
  }

  public boolean canSeeTarget() {
    return inst.getTable(limelightName).getEntry("tv").getDouble(0) == 1;
  }
}
