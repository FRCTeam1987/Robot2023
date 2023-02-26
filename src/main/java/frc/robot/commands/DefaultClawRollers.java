// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;

public class DefaultClawRollers extends CommandBase {

  private final Claw claw;
  private boolean isCube;
  private static final double rollerSpeed = 0.1;

  /** Creates a new DefaultClawRollers. */
  public DefaultClawRollers(Claw claw, boolean isCube) {
    this.claw = claw;
    this.isCube = isCube;
    addRequirements(this.claw);
  }

  public void setIsCube(boolean isCube) {
    this.isCube = isCube;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isCube && claw.getSpeedPercent() < rollerSpeed) {
      claw.setRollerSpeed(rollerSpeed);
    } else if (!isCube && claw.getSpeedPercent() != 0.0) {
      claw.setRollerSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
