package frc.robot.subsystems.vision;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean LOGGING = false;
  private boolean hasThrownException = false;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
    Constants.TAB_VISION.add(
        "Enable Estimator",
        new InstantCommand(
            () -> {
              this.LOGGING = true;
            }));
    Constants.TAB_MATCH.add(
        "Enable Estimator",
        new InstantCommand(
                () -> {
                  this.LOGGING = true;
                })
            .ignoringDisable(true));
    Constants.TAB_VISION.add(
        "Disable Estimator",
        new InstantCommand(
            () -> {
              this.LOGGING = false;
            }));
    Constants.TAB_MATCH.add(
        "Disable Estimator",
        new InstantCommand(
                () -> {
                  this.LOGGING = false;
                })
            .ignoringDisable(true));
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
    // System.out.println("vision periodic");
    VisionIOLimelightBase limelight = VisionIOLimelight.getInstance().getBestLimelight();
    try {
      double[] pose = limelight.getBotPose();
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(pose[0], pose[1], pose[2]),
              new Rotation3d(pose[3], pose[4], pose[5]));
      // System.out.println(pose2.getX() + ", " + pose2.getY() + ", " + pose2.getRotation().getDegrees());
      if (ADVANTAGE_KIT_ENABLED) {
        Logger.getInstance().recordOutput("Vision/RobotPose", pose3d);
      }
      // if (LOGGING) {
      final Rotation2d currentAngle = poseEstimator.getEstimatedPosition().getRotation();
      final Pose2d visionPose = new Pose2d(pose3d.toPose2d().getTranslation(), currentAngle);
      if (visionPose.getX() == 0.0 || visionPose.getY() == 0.0) {
        return;
      }
      poseEstimator.addVisionMeasurement(visionPose, pose[6]);
      // System.out.println(pose3d.toPose2d().toString());
      // }
    } catch (Exception ignored) {
      if (hasThrownException) {
        DriverStation.reportWarning(
            "Array out of bounds - Missing Limelight network tables entry", false);
        hasThrownException = true;
      }
    }
  }
}
