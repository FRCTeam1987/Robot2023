// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;

public class FlipWrist extends CommandBase {

  private final Wrist wrist;
  private final boolean rotation;

  public FlipWrist(final Wrist wrist, final boolean rotation) {
    this.wrist = wrist;
    this.rotation = rotation;
    addRequirements(this.wrist);
  }

  @Override
  public void initialize() {
    wrist.setRotation(rotation);
  }

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(wrist.getDegrees(), (rotation ? 180.0 : 0), 3));
  }
}
