// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.Wrist;

public class HomeWrist extends CommandBase { // README not tested do not use

  private static final double HOMING_PERCENT = -0.45;

  private final Wrist wrist;

  /** Creates a new HomeWrist. */
  public HomeWrist(final Wrist wrist) {
    this.wrist = wrist;
    addRequirements(this.wrist);
    Constants.TAB_MAIN.addNumber("WristPos", wrist::getPosition).withPosition(9, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPercent(HOMING_PERCENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("Home Wrist Interrupted: " + interrupted, false);
    if (interrupted) {
      return;
    }
    wrist.setPercent(0);
    DriverStation.reportWarning(wrist.getPosition() + " Before ConfigRelative", false);
    wrist.configRelative(Constants.INSTALLED_ARM.getWristOffset());
    DriverStation.reportWarning(wrist.getPosition() + " After ConfigRelative", false);
    wrist.setPosition(Wrist.ANGLE_STRAIGHT);
    DriverStation.reportWarning(wrist.getPosition() + " After SetPosition", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    DriverStation.reportWarning(
        "is Finished: " + wrist.hasHitHardstop() + ", position: " + wrist.getPosition(), false);
    return wrist.hasHitHardstop();
  }
}
