package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private DriverStation.Alliance lastAlliance = DriverStation.Alliance.Invalid;
  private SwerveDrivePoseEstimator poseEstimator;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

  }
  @Override
  public void periodic() {

    visionIO.updateInputs(io);
    Logger.getInstance().processInputs("Vision", io);

    VisionIOLimelightBase idealLimelight = VisionIOLimelight.getInstance().getIdealLimelight();
    Pose3d robotPose = VisionIOLimelight.getInstance().getIdealLimelight().getBotPose();

    if (poseEstimator
            .getEstimatedPosition()
            .minus(robotPose.toPose2d())
            .getTranslation()
            .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {
      poseEstimator.addVisionMeasurement(robotPose.toPose2d(), idealLimelight.getFrameMillis());

      Logger.getInstance().recordOutput("Vision/RobotPose", robotPose.toPose2d());
    }
  }
}