package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
  public static VisionIOLimelight getInstance() {
    return instance;
  }
  private static VisionIOLimelight instance;
  public static int row = 0;
  static List<VisionIOLimelightBase> limelights = new ArrayList<>();

  public VisionIOLimelight(String... limelightsIn) {
    instance = this;
    for (String name : limelightsIn) {
      if (!(NetworkTableInstance.getDefault().getTable(name).getEntry("camMode").getDouble(1987.42) == 1987.42)) {
        limelights.add(new VisionIOLimelightBase(name));
      } else {
        DriverStation.reportError("WARNING! '" + name + "' is not a valid/plugged in limelight!", false);
      }
    }
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    int size = limelights.size();
    //inputs.json = new String[size];
    inputs.frameTimes = new long[size];
    inputs.canSeeTag = new double[size];
    for (int i = 0; i < size; i++) {
      //inputs.json[i] = limelights.get(i).getRawJson();
      inputs.frameTimes[i] = limelights.get(i).getFrameMillis();
      inputs.canSeeTag[i] = limelights.get(i).canSeeTarget();
    }
  }
}
