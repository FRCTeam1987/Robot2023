// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class ArmProfile {
  public final int WRIST_OFFSET;
  public final double CANCODER_OFFSET;

  public ArmProfile(final int wristOffset, final double CANcoderOffset) {
    WRIST_OFFSET = wristOffset;
    CANCODER_OFFSET = CANcoderOffset;
  }

  public double getCancoderOffset() {
    return CANCODER_OFFSET;
  }

  public int getWristOffset() {
    return WRIST_OFFSET;
  }
}
