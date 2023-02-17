package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private SwerveDrivePoseEstimator poseEstimator;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
  }

  @Override
  public void periodic() {

    visionIO.updateInputs(io);
    Logger.getInstance().processInputs("Vision", io);

    VisionIOLimelightBase limelight = VisionIOLimelight.getInstance().getBestLimelight();
    Pose3dLatency pose = limelight.getBotPose();
    if (pose != null) {
      poseEstimator.addVisionMeasurement(pose.pose.toPose2d(), pose.latency);

      Logger.getInstance().recordOutput("Vision/RobotPose", pose.pose);
    }
  }
}
