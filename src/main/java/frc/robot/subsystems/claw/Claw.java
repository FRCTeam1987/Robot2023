package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.DefaultClawRollersSpin;
import frc.robot.commands.StopClawRollers;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public enum GamePiece {
    CONE,
    CUBE
  };
  private GamePiece gamePiece;

  private final double currentThreshold = 10.0; // amps

  /** Creates a new Claw. */
  public Claw(ClawIO io) {
    this.io = io;
    SmartDashboard.putData("Stop Claw", new StopClawRollers(this));
    SmartDashboard.putData("Run Claw", new CollectGamePiece(this));
  }

  public void initDefaultCommand() {
    setDefaultCommand(new DefaultClawRollersSpin(this));
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
  }
}
