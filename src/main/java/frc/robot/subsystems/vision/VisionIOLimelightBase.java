package frc.robot.subsystems.vision;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;
import static frc.robot.Constants.TAB_VISION;

import edu.wpi.first.networktables.*;

public class VisionIOLimelightBase {
  public final String limelightName;
  private final DoubleArraySubscriber botPoseSubscriber;
  private final StringSubscriber jsonSubscriber;

  public VisionIOLimelightBase(String limelightName) {
    this.limelightName = limelightName;

    String limelightNameShort = limelightName.replace("limelight-", "");
    NetworkTable inst = NetworkTableInstance.getDefault().getTable(limelightName);
    // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    TAB_VISION
        .addNumber(limelightNameShort + " count", this::getVisibleTagCount)
        .withPosition(0, VisionIOLimelight.row++);
    botPoseSubscriber = inst.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
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

  public double[] getBotPose() {
    return botPoseSubscriber.get();
  }
}
