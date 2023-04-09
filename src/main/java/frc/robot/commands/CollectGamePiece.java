// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class CollectGamePiece extends CommandBase {

  private final Claw CLAW;
  private final XboxController controller;
  private Debouncer DEBOUNCER;
  private boolean isCollected;

  private static final double COLLECTION_TIME = 0.15;
  private static final double CLAW_ROLLER_SPEED = -0.9;
  private static final double MAXIMUM_CURRENT = 27.5;

  private final GamePiece piece;

  public CollectGamePiece(final Claw claw, GamePiece piece) {
    this(claw, piece, null);
  }

  public CollectGamePiece(final Claw claw, GamePiece piece, XboxController controller) {
    this.CLAW = claw;
    this.piece = piece;
    this.controller = controller;
    DEBOUNCER = new Debouncer(COLLECTION_TIME);
    addRequirements(this.CLAW);
  }

  public boolean stopCondition() {
    // System.out.println("Current" + CLAW.getCurrent());
    return CLAW.getCurrent() > MAXIMUM_CURRENT;
  }

  @Override
  public void initialize() {
    DEBOUNCER = new Debouncer(COLLECTION_TIME);
    isCollected = DEBOUNCER.calculate(stopCondition());
    CLAW.setRollerSpeed(CLAW_ROLLER_SPEED);
  }

  @Override
  public void execute() {
    boolean shouldStop = stopCondition();
    isCollected = DEBOUNCER.calculate(shouldStop);
    if (controller != null) {
      if (shouldStop == true) {
        controller.setRumble(RumbleType.kBothRumble, 1);
      } else {
        controller.setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (piece == GamePiece.CUBE) {
      CLAW.setRollerSpeed(DefaultClawRollersSpin.CLAW_ROLLER_SPEED);
    } else {
      CLAW.stopRollers();
    }
    CLAW.setGamePiece(piece);
    if (controller != null) {
      controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return isCollected;
  }
}
