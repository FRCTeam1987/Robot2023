package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double[] currentAmps = new double[] {};
    public double[] currentVolts = new double[] {};
    public double armAbsoluteAngle;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default double getArmLength() {
    return 0.0;
  }

  default void setArmLength(double length) {}

  default double getArmAngle() {
    return 0.0;
  }

  default void setArmAngle(double angle) {}

  default void holdCurrentAngle() {}

  default void holdCurrentAngle(double desiredPosition) {}

  default void stallArm() {}

  default void setExtensionNominal() {}

  default void stop() {}

  default void zeroExtension() {}
}
