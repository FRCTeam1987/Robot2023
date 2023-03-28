// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class CollectGamePiece extends CommandBase {

  private final Claw CLAW;
  private Debouncer DEBOUNCER;
  private boolean isCollected;

  private static final double COLLECTION_TIME = 0.1;
  private static final double CLAW_ROLLER_SPEED = -0.9;
  private static final double MAXIMUM_CURRENT = 20;
  private final GamePiece piece;

  public CollectGamePiece(final Claw claw, GamePiece piece) {
    this.CLAW = claw;
    this.piece = piece;
    DEBOUNCER = new Debouncer(COLLECTION_TIME);
    addRequirements(this.CLAW);
  }

  public boolean stopCondition() {
    System.out.println("Current" + CLAW.getCurrent());
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
    isCollected = DEBOUNCER.calculate(stopCondition());
  }

  @Override
  public void end(boolean interrupted) {
    if(piece == GamePiece.CUBE) {
      CLAW.setRollerSpeed(DefaultClawRollersSpin.CLAW_ROLLER_SPEED);
    } else {
      CLAW.stopRollers();
    }
    CLAW.setGamePiece(piece);
  }

  @Override
  public boolean isFinished() {
    return isCollected;
  }
}
