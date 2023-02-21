package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final double currentThreshold = 10.0; // amps

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;
    SmartDashboard.putData(
        "norm",
        new InstantCommand(
            () -> {
              io.setPosition(true);
            }));
    SmartDashboard.putData(
        "invert",
        new InstantCommand(
            () -> {
              io.setPosition(false);
            }));

  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public void setPosition(boolean inverted) {
    io.setPosition(inverted);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Wrist", inputs);
  }
}
