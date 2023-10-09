package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  class WristIOInputs {
    public double currentAmps = 0.0;
    public double targetPositionRotations = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Sets position of wrist */
  void setRotation(boolean inverted);

  void setPosition(final int ticks);

  int getPosition();

  /** Return wrist degree */
  double getDegrees();

  double getCurrentAmps();

  void setPercent(final double percent);

  void configRelative(final int homeTicks);

  void incrementMidMatchOffset(final int ticks);
}
