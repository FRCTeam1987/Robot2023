package frc.robot.subsystems.claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  // private final Spark rollerMotor = new Spark(0);
  // private final CANSparkMax rollerMotor =
  // new CANSparkMax(Constants.ClawMotorID, CANSparkMax.MotorType.kBrushless);

  private final PIDController rollerPID =
      new PIDController(0.1, 0.01, 0.05); // magic numbers based off of google results
  private final double currentThreshold = 10.0; // amps

  ShuffleboardTab tab = Shuffleboard.getTab("ClawTab");

  /** Creates a new Claw. */
  public Claw(ClawIO io) {
    this.io = io;
  }

  public double getCurrent() {
    if (inputs.currentAmps.length > 0) {
      return inputs.currentAmps[0];
    } else {
      return 0.0;
    }
  }

  public boolean hasGamePiece() {
    return getCurrent() > currentThreshold;
  }

  public void stopRollers() {
    io.setRollerSpeed(0.0);
  }

  public void collectGamePiece() {
    io.setRollerSpeed(rollerPID.calculate(getCurrent()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
  }
}
