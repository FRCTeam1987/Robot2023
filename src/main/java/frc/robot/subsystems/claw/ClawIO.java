package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double currentAmps = 0.0;
    public double speedPercent = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Sets speed of roller motor */
  public default void setRollerSpeed(double speed) {}
}