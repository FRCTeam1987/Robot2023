// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class ArmProfile {
  private int wristOffset;
  private final double canCoderOffset;
  private int MatchOffset = 0;

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
    MatchOffset += incrementAmount;
    // DriverStation.reportWarning("-------- Modified MatchOffset!: " + MatchOffset, false);
  }

  public int getMatchOffset() {
    // DriverStation.reportWarning("- Returned MatchOffset!: " + MatchOffset, false);
    return MatchOffset;
  }
}
