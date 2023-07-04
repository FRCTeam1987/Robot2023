// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class ArmProfile {
  private int wristOffset;
  private final double canCoderOffset;

  public ArmProfile(final int wristOffset, final double CANcoderOffset) {
    this.wristOffset = wristOffset;
    canCoderOffset = CANcoderOffset;
  }

  public double getCancoderOffset() {
    return canCoderOffset;
  }

  public int getWristOffset() {
    return wristOffset;
  }

  public void addMatchOffset(int incrementAmount) {
    wristOffset += incrementAmount;
    DriverStation.reportWarning("-------- Modified MatchOffset!: " + wristOffset, false);
  }
}
