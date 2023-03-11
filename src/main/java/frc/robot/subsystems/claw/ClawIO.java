package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  class ClawIOInputs {
    public double currentAmps = 0.0;
    public double speedPercent = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ClawIOInputs inputs) {}

  /** Sets speed of roller motor */
  default void setRollerSpeed(double speed) {}
}
