// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class DefaultClawRollersSpin extends CommandBase {

  private final Claw claw;

  private static final double CLAW_ROLLER_SPEED = 0.1;

  public DefaultClawRollersSpin(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (claw.getGamePiece() == GamePiece.CONE && claw.getSpeedPercent() != 0.0) {
      claw.setRollerSpeed(0.0);
    } else if (claw.getGamePiece() == GamePiece.CUBE
        && claw.getSpeedPercent() < CLAW_ROLLER_SPEED) {
      claw.setRollerSpeed(CLAW_ROLLER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
