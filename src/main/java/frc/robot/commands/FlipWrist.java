// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;

public class FlipWrist extends CommandBase {

  private final Wrist WRIST;
  private final boolean ROTATION;

  public FlipWrist(final Wrist WRIST, final boolean ROTATION) {
    this.WRIST = WRIST;
    this.ROTATION = ROTATION;
  }

  @Override
  public void initialize() {
    WRIST.setRotation(ROTATION);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(
        WRIST.getDegrees(), (ROTATION ? 180.0 : 0), 3));
  }
}
