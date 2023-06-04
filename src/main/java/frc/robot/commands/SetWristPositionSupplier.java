// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;
import java.util.function.IntSupplier;

public class SetWristPositionSupplier extends CommandBase {
  private final IntSupplier position;
  private final Wrist wrist;
  /** Creates a new SetWristPosition. */
  public SetWristPositionSupplier(final Wrist wrist, final IntSupplier position) {
    this.wrist = wrist;
    this.position = position;
    addRequirements(this.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPosition(position.getAsInt());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(wrist.getPosition(), position.getAsInt(), 100);
  }
}
