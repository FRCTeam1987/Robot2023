package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

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

    VisionIOLimelightBase limelight = VisionIOLimelight.getInstance().getBestLimelight();
    Pose2d pose = limelight.getBotPose().toPose2d();
      if (pose != null) {
        poseEstimator.addVisionMeasurement(pose, limelight.getFrameMillis());

        Logger.getInstance().recordOutput("Vision/RobotPose", pose);
    }
  }
}