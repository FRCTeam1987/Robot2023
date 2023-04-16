// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    // m_limelights = (List<NetworkTable>) List.of(limelights).stream().map(limelight ->
    // NetworkTableInstance.getDefault().getTable(limelight));
    m_isPoseUpdateAllowed = isPoseUpdateAllowed;
    m_limelights = List.of(limelights);
    m_alliance = Alliance.Invalid;
    m_poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    //   TAB_MAIN2.addNumber(
    //       "estimated angle", () ->
    // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    //   TAB_MAIN2.addNumber("estimated x", () -> m_poseEstimator.getEstimatedPosition().getX());
    //   TAB_MAIN2.addNumber("estimated y", () -> m_poseEstimator.getEstimatedPosition().getY());
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
        || !Util.isWithinTolerance(pose.getY(), currentPose.getY(), 1.0)) {
      DriverStation.reportError("Ignoring pose beyond range", false); // UNCOMMENT ME FOR
      // DEBUGGING
      return;
    }
    if (pose.getX() > 4.75) {
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
      LimelightResults result = null;
      // List<String> llsWithTag = List.of();
      // final double tid = LimelightHelpers.getFiducialID(limelight);
      // if (tid < 1 || tid > 8) {
      //   DriverStation.reportWarning("Ignoring limelight: " + limelight + ", tid: " + tid, false);
      //   break;
      // }
      final double ta = LimelightHelpers.getTA(limelight);
      // if (ta <= 0.01) {
      //   DriverStation.reportWarning("ignoring too small of target", false);
      //   break;
      // }
      if (ta > 1.0) { // see how low this can go
        result = LimelightHelpers.getLatestResults(limelight);
        updatePose(result);
        // llsWithTag.add(limelight);
        break;
      }
      if (result == null) {
        result = LimelightHelpers.getLatestResults(limelight);
      }
      if (result != null && result.targetingResults.targets_Fiducials.length > 1) {
        updatePose(result);
        return;
      }
      // LimelightHelpers.getTA(limelight);
    }
    // final Optional<LimelightResults> llResultsWithMultipleTags =
    //     m_limelights.stream()
    //         .map(limelight -> LimelightHelpers.getLatestResults(limelight))
    //         .filter(llResult -> llResult.targetingResults.targets_Fiducials.length > 1)
    //         .findFirst();
    // if (llResultsWithMultipleTags.isEmpty()) {
    //   return;
    // }
    // final LimelightResults llresult = llResultsWithMultipleTags.get();
    // final Pose2d pose =
    //     m_alliance == Alliance.Red
    //         ? llresult.targetingResults.getBotPose2d_wpiRed()
    //         : llresult.targetingResults.getBotPose2d_wpiBlue();
    // final double latencySeconds =
    //     (llresult.targetingResults.latency_capture
    //             + llresult.targetingResults.latency_jsonParse
    //             + llresult.targetingResults.latency_pipeline)
    //         / 1000.0;

    // m_poseEstimator.addVisionMeasurement(
    //     pose,
    //     Timer.getFPGATimestamp() - latencySeconds);
    // return;

    /*
    final List<LimelightResults> llResultsWithTag =
        m_limelights.stream()
            .map(limelight -> LimelightHelpers.getLatestResults(limelight))
            .filter(llResult -> llResult.targetingResults.targets_Fiducials.length > 0)
            .collect(Collectors.toUnmodifiableList());
    if (llResultsWithTag.size() <= 0) {
      return;
    }
    // TODO do we want to average these out or just assume any LL with multiple tags is good to use.
    final List<LimelightResults> llResultsWithMultiTags =
        llResultsWithTag.stream().filter(llResult -> llResult.targetingResults.targets_Fiducials.length > 1).collect(Collectors.toUnmodifiableList());
    if (llResultsWithMultiTags.size() > 0) {
      updatePose(llResultsWithMultiTags);
      return;
    }

    // TODO we may want to find the standard deviation before updating pose from LL's with only one
    // tag.
    updatePose(llResultsWithTag);

    */
  }
}
