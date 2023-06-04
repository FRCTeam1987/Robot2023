// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Limelight;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PoseWithLatency {
  private final Pose2d mPose;
  private final double mLatency;

  public PoseWithLatency(final Pose2d pose, final double latency) {
    mPose = pose;
    mLatency = latency;
  }

  public Pose2d getPose() {
    return mPose;
  }

  public double getLatency() {
    return mLatency;
  }
}
