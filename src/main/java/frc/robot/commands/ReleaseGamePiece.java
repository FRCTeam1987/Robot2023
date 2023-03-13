package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;

public class ReleaseGamePiece extends CommandBase {

  private final Claw CLAW;
  private double startTime;

  private static final double RELEASE_TIME = 0.25;
  private static final double CLAW_ROLLER_SPEED = 0.75;

  public ReleaseGamePiece(final Claw claw) {
    this.CLAW = claw;
    addRequirements(this.CLAW);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if (CLAW.getGamePiece() == GamePiece.CONE) {
      CLAW.setRollerSpeed(CLAW_ROLLER_SPEED);
    } else if (CLAW.getGamePiece() == GamePiece.CUBE) {
      CLAW.setRollerSpeed(-CLAW_ROLLER_SPEED);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    CLAW.setRollerSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > RELEASE_TIME);
  }
}
