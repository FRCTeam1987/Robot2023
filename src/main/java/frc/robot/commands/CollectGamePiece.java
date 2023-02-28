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
  private final Debouncer DEBOUNCER;
  private boolean isCollected;

  private static final double COLLECTION_TIME = 0.25;
  private static final double CLAW_ROLLER_SPEED = 0.75;
  private static final double MAXIMUM_CURRENT = 35;

  public CollectGamePiece(final Claw claw) {
    this.CLAW = claw;
    DEBOUNCER = new Debouncer(COLLECTION_TIME);
    addRequirements(this.CLAW);
  }

  public boolean stopCondition() {
    return CLAW.getCurrent() > MAXIMUM_CURRENT;
  }

  @Override
  public void initialize() {
    isCollected = DEBOUNCER.calculate(stopCondition());
    if (CLAW.getGamePiece() == GamePiece.CONE) {
      CLAW.setRollerSpeed(-CLAW_ROLLER_SPEED);
    } else if (CLAW.getGamePiece() == GamePiece.CUBE) {
      CLAW.setRollerSpeed(CLAW_ROLLER_SPEED);
    }
  }

  @Override
  public void execute() {
    isCollected = DEBOUNCER.calculate(stopCondition());
  }

  @Override
  public void end(boolean interrupted) {
    CLAW.stopRollers();
  }

  @Override
  public boolean isFinished() {
    return isCollected;
  }
}
