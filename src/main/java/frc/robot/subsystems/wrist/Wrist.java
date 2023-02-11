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
    SmartDashboard.putData("Stop Wrist", new InstantCommand(() -> io.setRollerSpeed(0.0)));
    SmartDashboard.putData("Run Wrist", new InstantCommand(() -> io.setRollerSpeed(0.75)));
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public void stopRollers() {
    io.setRollerSpeed(0.0);
  }

  public void setRollerSpeed(double speed) { // speed -1.0 - 1.0
    io.setRollerSpeed(speed);
  }

  public double getSpeedPercent() {
    return inputs.speedPercent;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Wrist", inputs);
  }
}
