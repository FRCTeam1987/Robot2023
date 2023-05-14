package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;

public class ReleaseGamePiece extends CommandBase {

  private final Claw claw;
  private double startTime;

  private static final double RELEASE_TIME = 0.25;
  private static final double CLAW_ROLLER_SPEED = 0.75;

  public ReleaseGamePiece(final Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if (claw.getGamePiece() == GamePiece.CONE) {
      claw.setRollerSpeed(CLAW_ROLLER_SPEED);
    } else if (claw.getGamePiece() == GamePiece.CUBE) {
      claw.setRollerSpeed(-CLAW_ROLLER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    claw.stopRollers();
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > RELEASE_TIME);
  }
}
