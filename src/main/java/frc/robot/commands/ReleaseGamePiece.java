package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class ReleaseGamePiece extends CommandBase {

  private final Claw CLAW;
  private final GamePiece gamePiece;
  private double startTime;
  private boolean isReleased;
  
  private static final double RELEASE_TIME = 0.25;
  private static final double CLAW_ROLLER_SPEED = 0.75;
  private static final double MAXIMUM_CURRENT = 35;

  public ReleaseGamePiece(final Claw claw, GamePiece gamePiece) {
    this.CLAW = claw;
    this.gamePiece = gamePiece;
    addRequirements(this.CLAW);
  }

  public boolean stopCondition() {
    return CLAW.getCurrent() > MAXIMUM_CURRENT;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if (gamePiece == GamePiece.CONE) CLAW.setRollerSpeed(CLAW_ROLLER_SPEED);
    else if (gamePiece == GamePiece.CUBE) CLAW.setRollerSpeed(-CLAW_ROLLER_SPEED);
  }

  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime > 0) isReleased = true;
  }

  @Override
  public void end(boolean interrupted) {
    CLAW.stopRollers();
  }

  @Override
  public boolean isFinished() {
    return isReleased;
  }
}