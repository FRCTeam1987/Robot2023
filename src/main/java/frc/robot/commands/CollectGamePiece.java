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

  private final Claw claw;
  private final XboxController controller;
  private Debouncer debouncer;
  private boolean isCollected;

  private static final double COLLECTION_TIME = 0.1;
  private static final double CLAW_ROLLER_SPEED = 0.95;
  private static final double MAXIMUM_CURRENT = 30.0;
  private static final double MAXIMUM_CURRENT_CUBE = 12.0;

  private final GamePiece piece;

  public CollectGamePiece(final Claw claw, GamePiece piece) {
    this(claw, piece, null);
  }

  public CollectGamePiece(final Claw claw, GamePiece piece, XboxController controller) {
    this.claw = claw;
    this.piece = piece;
    this.controller = controller;
    debouncer = new Debouncer(COLLECTION_TIME);
    addRequirements(this.claw);
  }

  public boolean stopCondition() {
    return claw.getCurrent()
        > (this.piece == GamePiece.CONE
            ? MAXIMUM_CURRENT
            : MAXIMUM_CURRENT_CUBE); // TODO flex this based on game piece type?
  }

  @Override
  public void initialize() {
    debouncer = new Debouncer(COLLECTION_TIME);
    isCollected = debouncer.calculate(stopCondition());
    claw.setRollerSpeed(piece == GamePiece.CONE ? CLAW_ROLLER_SPEED : -CLAW_ROLLER_SPEED);
  }

  @Override
  public void execute() {
    boolean shouldStop = stopCondition();
    isCollected = debouncer.calculate(shouldStop);
    if (controller != null) {
      if (shouldStop) {
        controller.setRumble(RumbleType.kBothRumble, 1);
      } else {
        controller.setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (piece == GamePiece.CUBE) {
      claw.setRollerSpeed(DefaultClawRollersSpin.CLAW_ROLLER_SPEED);
    } else {
      claw.stopRollers();
    }
    claw.setGamePiece(piece);
    if (controller != null) {
      controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return isCollected;
  }
}
