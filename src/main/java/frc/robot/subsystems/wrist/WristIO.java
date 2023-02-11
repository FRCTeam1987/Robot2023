package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double currentAmps = 0.0;
    public double speedPercent = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets speed of roller motor */
  public default void setRollerSpeed(double speed) {}
}
