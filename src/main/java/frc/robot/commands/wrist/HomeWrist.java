// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.Wrist;

public class HomeWrist extends CommandBase {

  private static final double HOMING_PERCENT = -0.2;
  private static final double HOMING_CURRENT_THRESHOLD = 20;

  private final Wrist wrist;

  /** Creates a new HomeWrist. */
  public HomeWrist(final Wrist wrist) {
    this.wrist = wrist;
    addRequirements(this.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPercent(HOMING_PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.configRelative(Constants.WRIST_OFFSET);
    wrist.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.getCurrent() > HOMING_CURRENT_THRESHOLD;
  }
}
