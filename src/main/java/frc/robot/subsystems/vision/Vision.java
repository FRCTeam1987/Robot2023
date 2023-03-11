package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private final SwerveDrivePoseEstimator poseEstimator;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
  }

  @Override
  public void periodic() {

    visionIO.updateInputs(io);
    Logger.getInstance().processInputs("Vision", io);

    VisionIOLimelightBase limelight = VisionIOLimelight.getInstance().getBestLimelight();
    try {
      double[] pose = limelight.getBotPose();
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(pose[0], pose[1], pose[2]),
              new Rotation3d(pose[3], pose[4], pose[5]));
      Logger.getInstance().recordOutput("Vision/RobotPose", pose3d);
      poseEstimator.addVisionMeasurement(pose3d.toPose2d(), pose[6]);
    } catch (Exception ignored) {

    }
  }
}
