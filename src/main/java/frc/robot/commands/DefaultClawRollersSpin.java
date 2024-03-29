// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;

public class DefaultClawRollersSpin extends CommandBase {

  private final Claw claw;

  public static final double CONE_ROLLER_SPEED = 0.1;
  public static final double CUBE_ROLLER_SPEED = -0.05;
  public static final double CLAW_ROLLER_SPEED = -0.05;

  public DefaultClawRollersSpin(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    // GamePiece gamePiece = claw.getGamePiece();
    // double speed = claw.getSpeedPercent();
    claw.setRollerSpeed(claw.isCone() ? CONE_ROLLER_SPEED : CUBE_ROLLER_SPEED);

    // if (gamePiece == GamePiece.CONE && speed != 0.0) {
    //   claw.setRollerSpeed(0.0);
    // } else if (gamePiece == GamePiece.CUBE) {
    //   claw.setRollerSpeed(CLAW_ROLLER_SPEED);
    // } else if (gamePiece == GamePiece.NONE && speed != 0.0) {
    //   claw.stopRollers();
    // }
  }
}
