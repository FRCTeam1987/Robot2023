// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;

public class SetWristPosition extends CommandBase {
  private final int position;
  private final Wrist wrist;
  /** Creates a new SetWristPosition. */
  public SetWristPosition(int position, Wrist wrist) {
    this.wrist = wrist;
    this.position = position;
    addRequirements(this.wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // DriverStation.reportWarning("==== Should Go Home ", false);
    wrist.setPosition(position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(wrist.getPosition(), position, 100);
  }
}
