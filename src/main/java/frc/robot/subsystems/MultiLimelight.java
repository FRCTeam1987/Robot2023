// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.lib.Limelight.LimelightHelpers.LimelightResults;
import frc.lib.Limelight.PoseWithLatency;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.util.Util;
import java.util.List;
import java.util.function.BooleanSupplier;

// TODO none of this is tested

public class MultiLimelight extends SubsystemBase {

  public static double calculateStandardDeviation(double input_array[]) {
    double sum = 0.0, standard_deviation = 0.0;
    int array_length = input_array.length;
    for (double temp : input_array) {
      sum += temp;
    }
    double mean = sum / array_length;
    for (double temp : input_array) {
      standard_deviation += Math.pow(temp - mean, 2);
    }
    return Math.sqrt(standard_deviation / array_length);
  }

  private Alliance m_alliance;
  private final BooleanSupplier m_isPoseUpdateAllowed;
  private final List<String> m_limelights;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  public static final ShuffleboardTab TAB_MAIN2 = Shuffleboard.getTab("Main2");

  /**
   * Creates a new MultiLimelight.
   *
   * @param isPoseUpdateAllowed - True if allowed, false otherwise. Intent is to disallow in some
   *     circumstances like driving too fast.
   * @param limelights - Any number of limelight names to use.
   */
  public MultiLimelight(BooleanSupplier isPoseUpdateAllowed, String... limelights) {
    m_isPoseUpdateAllowed = isPoseUpdateAllowed;
    m_limelights = List.of(limelights);
    m_alliance = Alliance.Invalid;
    m_poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    for (String m_limelight : m_limelights) {
      LimelightHelpers.getLatestResults(m_limelight);
    }
  }

  private void updatePose(final List<LimelightResults> llResults) {
    final PoseWithLatency averagePose =
        llResults.stream()
            .map(
                llResult ->
                    new PoseWithLatency(
                        getAlliancePose(llResult),
                        llResult.targetingResults.latency_capture
                            + llResult.targetingResults.latency_pipeline
                            + llResult.targetingResults.latency_jsonParse))
            .reduce(
                null,
                (accumulator, current) -> {
                  if (accumulator == null) {
                    return current;
                  }
                  return new PoseWithLatency(
                      new Pose2d(
                          (accumulator.getPose().getX() + current.getPose().getX()) / 2.0,
                          (accumulator.getPose().getY() + current.getPose().getY()) / 2.0,
                          Rotation2d.fromDegrees(
                              (accumulator.getPose().getRotation().getDegrees()
                                      + current.getPose().getRotation().getDegrees())
                                  / 2.0)),
                      (accumulator.getLatency() + current.getLatency()) / 2.0);
                });
    // TODO we may want to avoid updating the pose rotation... just x & y, use gyro as rotation;
    // some teams are doing this
    // TODO we may want to check the pose at latency time to make sure the pose from vision is
    // "close enough" before we update
    m_poseEstimator.addVisionMeasurement(
        averagePose.getPose(), Timer.getFPGATimestamp() - averagePose.getLatency() / 1000);
  }

  private Pose2d getAlliancePose(final LimelightResults llresult) {
    if (m_alliance == Alliance.Red) {
      return llresult.targetingResults.getBotPose2d_wpiRed();
    }
    if (m_alliance == Alliance.Blue) {
      return llresult.targetingResults.getBotPose2d_wpiBlue();
    }
    return null; // TODO WHAT DO WE DO HERE?
  }

  public void updatePose(LimelightResults result) {
    if (m_alliance == Alliance.Invalid) {
      return;
    }
    final Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    if (currentPose.getX() > 3.5 && currentPose.getX() < 4.4) {
      DriverStation.reportError("Ignoring pose on bump", false);
      return;
    }
    final Pose2d pose =
        m_alliance == Alliance.Red
            ? result.targetingResults.getBotPose2d_wpiRed()
            : result.targetingResults.getBotPose2d_wpiBlue();
    final double latencySeconds =
        (result.targetingResults.latency_capture
                + result.targetingResults.latency_jsonParse
                + result.targetingResults.latency_pipeline)
            / 1000.0;
    // play with seeing how far we want to allow a pose update
    if (!Util.isWithinTolerance(pose.getX(), currentPose.getX(), 1.0)
        || !Util.isWithinTolerance(pose.getY(), currentPose.getY(), 1.0)
        || pose.getY() < 0.4
        || pose.getX() < 1.75
        || (pose.getX() > 3.5 && pose.getX() < 4.4)) {
      DriverStation.reportError("Ignoring pose beyond range", false); // UNCOMMENT ME FOR
      // DEBUGGING
      return;
    }
    if (pose.getX() > 4.8) {
      DriverStation.reportWarning("Ignoring too far pose", false);
      return;
    }
    m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds);
    DriverStation.reportError("Updated pose! x: " + pose.getX() + ", y: " + pose.getY(), false);
    // //UNCOMMENT ME FOR DEBUGGING
  }

  public void updatePoseNoResults(Pose2d pose, double timing) {
    if (m_alliance == Alliance.Invalid) {
      return;
    }
    final Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    if (currentPose.getX() > 3.5 && currentPose.getX() < 4.4) {
      DriverStation.reportError("Ignoring pose on bump", false);
      return;
    }
    final double latencySeconds = timing;
    // play with seeing how far we want to allow a pose update
    if (!Util.isWithinTolerance(pose.getX(), currentPose.getX(), 1.0)
        || !Util.isWithinTolerance(pose.getY(), currentPose.getY(), 1.0)
        || pose.getY() < 0.4
        || pose.getX() < 1.75
        || (pose.getX() > 3.5 && pose.getX() < 4.4)) {
      DriverStation.reportError("Ignoring pose beyond range", false); // UNCOMMENT ME FOR
      // DEBUGGING
      return;
    }
    if (pose.getX() > 4.8) {
      DriverStation.reportWarning("Ignoring too far pose", false);
      return;
    }
    m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds);
    DriverStation.reportError("Updated pose! x: " + pose.getX() + ", y: " + pose.getY(), false);
    // //UNCOMMENT ME FOR DEBUGGING
  }

  @Override
  public void periodic() {
    if (m_alliance == Alliance.Invalid) {
      m_alliance = DriverStation.getAlliance();
      return;
    }
    if (!m_isPoseUpdateAllowed.getAsBoolean()) {
      return;
    }
    for (String limelight : m_limelights) {
      int countTags = 0;
      countTags =
          (int)
              LimelightHelpers.getJSONDump(limelight).codePoints().filter(ch -> ch == 'm').count();

      if (countTags > 1) {
        // updatePose(result);
        double[] pose =
            m_alliance == Alliance.Blue
                ? LimelightHelpers.getBotPose_wpiBlue(limelight)
                : LimelightHelpers.getBotPose_wpiRed(limelight);
        Pose2d pose2d =
            new Pose3d(
                    new Translation3d(pose[0], pose[1], pose[2]),
                    new Rotation3d(pose[3], pose[4], pose[5]))
                .toPose2d();
        updatePoseNoResults(pose2d, pose[6] / 1000);
        return;
      }
    }
  }
}
