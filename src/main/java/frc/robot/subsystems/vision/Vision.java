package frc.robot.subsystems.vision;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean hasThrownException = false;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
  }

  @Override
  public void periodic() {
    if (ADVANTAGE_KIT_ENABLED) {
      try {
        visionIO.updateInputs(io);
        Logger.getInstance().processInputs("Vision", io);
      } catch (Exception ignored) {
      }
    }
    VisionIOLimelightBase limelight = VisionIOLimelight.getInstance().getBestLimelight();
    try {
      double[] pose = limelight.getBotPose();
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(pose[0], pose[1], pose[2]),
              new Rotation3d(pose[3], pose[4], pose[5]));
      if (ADVANTAGE_KIT_ENABLED) {
        Logger.getInstance().recordOutput("Vision/RobotPose", pose3d);
      }
      final Rotation2d currentAngle = poseEstimator.getEstimatedPosition().getRotation();
      final Pose2d visionPose = new Pose2d(pose3d.toPose2d().getTranslation(), currentAngle);
      if (visionPose.getX() == 0.0 || visionPose.getY() == 0.0) {
        return;
      }
      poseEstimator.addVisionMeasurement(visionPose, pose[6]);
    } catch (Exception ignored) {
      if (hasThrownException) {
        DriverStation.reportWarning(
            "Array out of bounds - Missing Limelight network tables entry", false);
        hasThrownException = true;
      }
    }
  }
}
