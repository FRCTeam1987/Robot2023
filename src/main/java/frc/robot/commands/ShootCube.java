// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class ShootCube extends CommandBase {

  private static final double CLAW_ROLLER_SPEED = 0.9;

  private final Claw claw;

  /** Creates a new EjectGamePiece. */
  public ShootCube(final Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    claw.setRollerSpeed(CLAW_ROLLER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopRollers();
    claw.setGamePiece(GamePiece.NONE);
  }
}
