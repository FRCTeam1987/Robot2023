package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

    VisionIOLimelightBase bestLimelight = VisionIOLimelight.getInstance().getBestLimelight();
    Pose3d robotPose = VisionIOLimelight.getInstance().getBestLimelight().getBotPose();

    if (robotPose != null && poseEstimator
            .getEstimatedPosition()
            .minus(robotPose.toPose2d())
            .getTranslation()
            .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {
      poseEstimator.addVisionMeasurement(robotPose.toPose2d(), bestLimelight.getFrameMillis());

      Logger.getInstance().recordOutput("Vision/RobotPose", robotPose.toPose2d());
    }
  }
}