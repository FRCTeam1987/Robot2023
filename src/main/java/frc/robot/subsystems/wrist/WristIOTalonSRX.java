package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class WristIOTalonSRX implements WristIO {

  private final TalonSRX clawRollerMotor;

  public WristIOTalonSRX(int clawRollerMotorID) {
    clawRollerMotor = new TalonSRX(clawRollerMotorID);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = clawRollerMotor.getStatorCurrent();
    inputs.speedPercent = clawRollerMotor.getSelectedSensorVelocity();
  }

  public void setRollerSpeed(double speed) {
    clawRollerMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }
}
