// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.util.Util;
import java.util.List;
import java.util.function.BooleanSupplier;

public class MultiLimelight extends SubsystemBase {

  public static double calculateStandardDeviation(double[] inputArray) {
    double sum = 0.0;
    double standardDeviation = 0.0;
    int arrayLength = inputArray.length;
    for (double temp : inputArray) {
      sum += temp;
    }
    double mean = sum / arrayLength;
    for (double temp : inputArray) {
      standardDeviation += Math.pow(temp - mean, 2);
    }
    return Math.sqrt(standardDeviation / arrayLength);
  }

  private Alliance mAlliance;
  private final BooleanSupplier mIsPoseUpdateAllowed;
  private final List<String> mLimelights;
  private final SwerveDrivePoseEstimator mPoseEstimator;
  public static final ShuffleboardTab TAB_MAIN = Shuffleboard.getTab("Main");

  /**
   * Creates a new MultiLimelight.
   *
   * @param isPoseUpdateAllowed - True if allowed, false otherwise. Intent is to disallow in some
   *     circumstances like driving too fast.
   * @param limelights - Any number of limelight names to use.
   */
  public MultiLimelight(BooleanSupplier isPoseUpdateAllowed, String... limelights) {
    mIsPoseUpdateAllowed = isPoseUpdateAllowed;
    mLimelights = List.of(limelights);
    mAlliance = Alliance.Invalid;
    mPoseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    for (String m_limelight : mLimelights) {
      LimelightHelpers.getLatestResults(m_limelight);
    }
  }

  public void updatePose(LimelightResults result) {
    if (mAlliance == Alliance.Invalid) {
      return;
    }
    final Pose2d currentPose = mPoseEstimator.getEstimatedPosition();
    if (currentPose.getX() > 3.5 && currentPose.getX() < 4.4) {
      DriverStation.reportError("Ignoring pose on bump", false);
      return;
    }
    final Pose2d pose =
        mAlliance == Alliance.Red
            ? result.targetingResults.getBotPose2dWpiRed()
            : result.targetingResults.getBotPose2dWpiBlue();
    final double latencySeconds =
        (result.targetingResults.latencyCapture
                + result.targetingResults.latencyjsonParse
                + result.targetingResults.latencyPipeline)
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
    mPoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds);
    DriverStation.reportError("Updated pose! x: " + pose.getX() + ", y: " + pose.getY(), false);
    // //UNCOMMENT ME FOR DEBUGGING
  }

  public void updatePoseNoResults(Pose2d pose, double timing) {
    if (mAlliance == Alliance.Invalid) {
      return;
    }
    final Pose2d currentPose = mPoseEstimator.getEstimatedPosition();
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
    mPoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds);
    DriverStation.reportError("Updated pose! x: " + pose.getX() + ", y: " + pose.getY(), false);
    // //UNCOMMENT ME FOR DEBUGGING
  }

  @Override
  public void periodic() {
    if (mAlliance == Alliance.Invalid) {
      mAlliance = DriverStation.getAlliance();
      return;
    }
    if (!mIsPoseUpdateAllowed.getAsBoolean()) {
      return;
    }
    for (String limelight : mLimelights) {
      int countTags = 0;
      countTags =
          (int)
              LimelightHelpers.getJSONDump(limelight).codePoints().filter(ch -> ch == 'm').count();

      if (countTags > 1) {
        double[] pose =
            mAlliance == Alliance.Blue
                ? LimelightHelpers.getBotPoseWpiBlue(limelight)
                : LimelightHelpers.getBotPoseWpiRed(limelight);
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
