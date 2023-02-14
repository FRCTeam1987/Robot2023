package frc.robot.subsystems.wrist;

import java.nio.channels.WritableByteChannel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class WristIOTalonSRX implements WristIO {

  private final TalonSRX wristMotor;
  

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.feedbackNotContinuous = true;
    wristMotor.configFactoryDefault();
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = wristMotor.getStatorCurrent();
    inputs.targetPositionRotations = wristMotor.getSelectedSensorPosition();
  }

  public void setWristSpeed(double speed) {
    wristMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setPosition(double targetPositionRotations) { // in Ticks
    wristMotor.set(ControlMode.Position, targetPositionRotations);
  }

  public double getDegree() {
    return (double) wristMotor.getSensorCollection().getPulseWidthPosition() /  4096.0 * 360.0;

  }
}
