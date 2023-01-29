// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerClaw;

public class PlaceGamePiece extends CommandBase {
  /** Creates a new PlaceGamePiece. */
  public PlaceGamePiece() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RollerClaw.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RollerClaw.getInstance().runRollerOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RollerClaw.getInstance().stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
