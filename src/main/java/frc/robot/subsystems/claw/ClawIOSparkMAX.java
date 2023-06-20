package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class ClawIOSparkMAX implements ClawIO {

  private final CANSparkMax clawRollerMotor;

  public ClawIOSparkMAX(int clawRollerMotorID) {
    clawRollerMotor = new CANSparkMax(clawRollerMotorID, CANSparkMax.MotorType.kBrushless);
    clawRollerMotor.restoreFactoryDefaults();
    clawRollerMotor.setIdleMode(IdleMode.kBrake);
    clawRollerMotor.setSmartCurrentLimit(40);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.currentAmps = clawRollerMotor.getOutputCurrent();
    inputs.speedPercent = clawRollerMotor.getEncoder().getVelocity();
  }

  @Override
  public double getCurrentAmps() {
    return clawRollerMotor.getOutputCurrent();
  }

  @Override
  public double getSpeedPercent() {
    return clawRollerMotor.getOutputCurrent();
  }

  @Override
  public void setRollerSpeed(double speed) {
    clawRollerMotor.set(speed);
  }
}
