package frc.robot.subsystems.vision2;

import static frc.robot.Constants.LIMELIGHT_PIPELINE;
import static frc.robot.Constants.TAB_VISION;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightBase {
  public final String limelightName;
  private final DoubleArraySubscriber botPoseSubscriber;
  private StringSubscriber jsonSubscriber;
  private final DoubleSubscriber targetAreaSubscriber;
  private Alliance m_alliance;

  public LimelightBase(String limelightName) {
    this.limelightName = limelightName;
    m_alliance = Alliance.Invalid;

    NetworkTable inst = NetworkTableInstance.getDefault().getTable(limelightName);
    // inst.getEntry("pipeline").setNumber(LIMELIGHT_PIPELINE);
    botPoseSubscriber = inst.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    jsonSubscriber = inst.getStringTopic("json").subscribe("[]");
    targetAreaSubscriber = inst.getDoubleTopic("ta").subscribe(0.0);  // > 2.5 is pretty stable
  }

  public String getRawJson() {
    return jsonSubscriber.get();
  }

  /**
   * This is cheeky and may break in the future. Each fiducial object in the json array contains
one
   * instance of the letter "m". We parse through the entire json string and grab the count of
the
   * letter m.
   */
  public int getVisibleTagCount() {
    return (int) jsonSubscriber.get().codePoints().filter(ch -> ch == 'm').count();
  }

  public double getTargetArea() {
    return targetAreaSubscriber.get();
  }

  public double[] getBotPose() {
    if (m_alliance == Alliance.Invalid) {
      m_alliance = DriverStation.getAlliance();
      return null;
    }
    return botPoseSubscriber.get();
  }
}
