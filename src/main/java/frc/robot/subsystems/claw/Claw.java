package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.EjectGamePiece;
import frc.robot.commands.StopClawRollers;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public enum GamePiece {
    CONE,
    CUBE,
    NONE
  };

  private GamePiece gamePiece = GamePiece.CONE;

  private final double currentThreshold = 10.0; // amps

  /** Creates a new Claw. */
  public Claw(ClawIO io) {
    this.io = io;
    SmartDashboard.putData("Stop Claw", new StopClawRollers(this));
    SmartDashboard.putData("Collect Cube", new CollectGamePiece(this, GamePiece.CUBE));
    SmartDashboard.putData("Collect Cone", new CollectGamePiece(this, GamePiece.CONE));
    SmartDashboard.putData("Switch game piece", new InstantCommand(() -> this.changeGamePiece()));
    SmartDashboard.putData("Run Claw Plain", new InstantCommand(() -> setRollerSpeed(0.75), this));
    SmartDashboard.putData("Eject Game Piece", new EjectGamePiece(this).withTimeout(0.25));
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

  public void changeGamePiece() {
    switch (gamePiece) {
      case CONE:
        setCube();
        break;
      case CUBE:
        setCone();
        break;
      default:
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
    return inputs.currentAmps;
  }

  public double getSpeedPercent() {
    return inputs.speedPercent;
  }

  public boolean hasGamePiece() {
    return getCurrent() > currentThreshold;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    SmartDashboard.putBoolean("is cone", this.isCone());
  }
}
