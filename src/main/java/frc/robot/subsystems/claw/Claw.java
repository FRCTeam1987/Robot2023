package frc.robot.subsystems.claw;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public enum GamePiece {
    CONE,
    CUBE,
    NONE
  }

  private GamePiece gamePiece = GamePiece.CONE;

  private static final double CURRENT_THRESHOLD = 10.0; // amps

  /** Creates a new Claw. */
  public Claw(ClawIO io) {
    this.io = io;
  }

  public void setCone() {
    gamePiece = GamePiece.CONE;
  }

  public void setCube() {
    gamePiece = GamePiece.CUBE;
  }

  public void setGamePiece(final GamePiece desiredGamePiece) {
    gamePiece = desiredGamePiece;
  }

  public boolean isCone() {
    return gamePiece == GamePiece.CONE;
  }

  public boolean isCube() {
    return gamePiece == GamePiece.CUBE;
  }

  public void changeGamePiece() {
    if (Objects.requireNonNull(gamePiece) == GamePiece.CUBE) {
      setCone();
    } else {
      setCube();
    }
  }

  public void setRollerSpeed(double speedPercent) {
    io.setRollerSpeed(speedPercent);
  }

  public void stopRollers() {
    io.setRollerSpeed(0.0);
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

  public double getCurrent() {
    return io.getCurrentAmps();
  }

  public double getSpeedPercent() {
    return io.getSpeedPercent();
  }

  public boolean hasGamePiece() {
    return getCurrent() > CURRENT_THRESHOLD;
  }

  @Override
  public void periodic() {
    if (ADVANTAGE_KIT_ENABLED) {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Claw", inputs);
    }
  }
}
