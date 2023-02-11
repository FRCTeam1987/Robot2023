package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.StopClawRollers;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  private final double currentThreshold = 10.0; // amps

  /** Creates a new Claw. */
  public Claw(ClawIO io) {
    this.io = io;
    SmartDashboard.putData("Stop Claw", new StopClawRollers(this));
    SmartDashboard.putData("Run Claw", new CollectGamePiece(this));
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public boolean hasGamePiece() {
    return getCurrent() > currentThreshold;
  }

  public void stopRollers() {
    io.setRollerSpeed(0.0);
  }

  public void collectGamePiece() {
    io.setRollerSpeed(0.5);
  }

  public void stallRollers() {
    io.setRollerSpeed(0.085);
  }

  public void setRollerSpeed(double speed) { // speed -1.0 - 1.0
    io.setRollerSpeed(speed);
  }

  public double getSpeedPercent() {
    return inputs.speedPercent;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
  }
}
