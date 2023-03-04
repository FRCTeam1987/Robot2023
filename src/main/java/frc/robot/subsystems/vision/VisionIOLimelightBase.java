package frc.robot.subsystems.vision;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionIOLimelightBase {
  public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("Limelights");
  public final String limelightName;
  private DoubleArraySubscriber botPoseSubscriber;
  private final StringSubscriber jsonSubscriber;
  private final NetworkTable inst;
  public VisionIOLimelightBase(String limelightName) {
    int column = 0;
    this.limelightName = limelightName;

    String limelightNameShort = limelightName.replace("limelight-", "");
    inst = NetworkTableInstance.getDefault().getTable(limelightName);
    // inst.getTable(limelightName).getEntry("ledMode").setNumber(1);
    inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    LIMELIGHT_TAB
        .addNumber(limelightNameShort + " count", this::getVisibleTagCount)
        .withPosition(column++, VisionIOLimelight.row++);
    botPoseSubscriber = inst.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
  }

  public String getRawJson() {
    return jsonSubscriber.get();
  }

  /**
   * Set the Robot's Alliance for Auto Pathing
   * @param botPoseType True = Blue, False = Red
   */
  public void setAlliance(boolean botPoseType) {
    botPoseSubscriber = inst.getDoubleArrayTopic(botPoseType ? "botpose_wpiblue" : "botpose_wpired").subscribe(new double[] {});
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
