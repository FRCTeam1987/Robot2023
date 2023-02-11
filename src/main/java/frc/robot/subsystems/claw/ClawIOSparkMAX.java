package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

public class ClawIOSparkMAX implements ClawIO {

  private final CANSparkMax clawRollerMotor;

  public ClawIOSparkMAX(int clawRollerMotorID) {
    clawRollerMotor = new CANSparkMax(clawRollerMotorID, CANSparkMax.MotorType.kBrushless);
    clawRollerMotor.restoreFactoryDefaults();
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.currentAmps = clawRollerMotor.getOutputCurrent();
    inputs.speedPercent = clawRollerMotor.getEncoder().getVelocity();
  }

  public void setRollerSpeed(double speed) {
    clawRollerMotor.set(speed);
  }
}
