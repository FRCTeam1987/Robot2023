package frc.robot.subsystems;

import static frc.robot.Constants.TAB_CLAW;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.EjectGamePiece;
import frc.robot.commands.StopClawRollers;
import java.util.Objects;

public class Claw extends SubsystemBase {
  private final CANSparkMax clawRollerMotor;

  public enum GamePiece {
    CONE,
    CUBE,
    NONE
  }

  private GamePiece gamePiece = GamePiece.CONE;

  // TODO: [MARKER] Make this a constant
  private final double currentThreshold = 10.0; // amps

  /** Creates a new Claw. */
  public Claw(int clawRollerMotorID) {
    TAB_CLAW.add("Stop Claw", new StopClawRollers(this));
    // TAB_CLAW.add("Collect Cube", new CollectGamePiece(this, GamePiece.CUBE));
    // TAB_CLAW.add("Collect Cone", new CollectGamePiece(this, GamePiece.CONE));
    // TAB_CLAW.add("Switch game piece", new InstantCommand(() ->
    // this.changeGamePiece()));
    TAB_CLAW.add("Run Claw Plain", new InstantCommand(() -> setRollerSpeed(0.75), this));
    TAB_CLAW.add("Eject Game Piece", new EjectGamePiece(this).withTimeout(0.25));

    clawRollerMotor = new CANSparkMax(clawRollerMotorID, CANSparkMax.MotorType.kBrushless);
    clawRollerMotor.restoreFactoryDefaults();
    clawRollerMotor.setIdleMode(IdleMode.kBrake);
    // TODO ADD CURRENT LIMIT TO PROTECT MOTOR
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
    if (Objects.requireNonNull(gamePiece) == GamePiece.CUBE) {
      setCone();
    } else {
      setCube();
    }
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

  public boolean hasGamePiece() {
    return getCurrentAmps() > currentThreshold;
  }

  public double getCurrentAmps() {
    return clawRollerMotor.getOutputCurrent();
  }

  public double getSpeedPercent() {
    return clawRollerMotor.getOutputCurrent();
  }

  public void setRollerSpeed(double speed) {
    clawRollerMotor.set(speed);
  }

  @Override
  public void periodic() {}
}
