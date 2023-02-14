package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double currentAmps = 0.0;
    public double targetPositionRotations = 0.0;
    public double getDegree = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets position of wrist */
  public default void setPosition(double targetPositionRotations) {}

  /** Return wrist degree */
  public default double getDegree() {return getDegree();}
}
