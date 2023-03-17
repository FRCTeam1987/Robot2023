package frc.robot.subsystems.wrist;

import static frc.robot.Constants.TAB_WRIST;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class WristIOTalonSRX implements WristIO {
  public static final int ANGLE_STRAIGHT = 1457 + Constants.WRIST_OFFSET;

  private final WPI_TalonSRX wristMotor;

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.motionAcceleration = 4000;
    wristConfig.motionCruiseVelocity = 6000;
    wristConfig.feedbackNotContinuous = true;
    wristConfig.slot0.kP = 4.0;
    wristConfig.slot0.kD = 0.0;
    wristConfig.slot0.allowableClosedloopError = 0;
    wristConfig.neutralDeadband = 0.001;
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(wristConfig);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(6);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setInverted(InvertType.InvertMotorOutput);
    wristMotor.setSensorPhase(true);
    wristMotor.configContinuousCurrentLimit(15);
    wristMotor.configPeakCurrentLimit(30);
    // setPosition(ANGLE_STRAIGHT);
    TAB_WRIST.add(
        "reset",
        new InstantCommand(
            () -> {
              wristMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
              wristMotor.setSelectedSensorPosition(0);
            }));
    TAB_WRIST.addNumber("Current Position", wristMotor::getSelectedSensorPosition);
    TAB_WRIST.addNumber("Motor Voltage", wristMotor::getMotorOutputVoltage);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = wristMotor.getStatorCurrent();
    inputs.targetPositionRotations = wristMotor.getSelectedSensorPosition();
  }

  public void setPosition(final int ticks) {
    wristMotor.set(TalonSRXControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, -0.1);
  }

  public int getPosition() {
    return (int) wristMotor.getSelectedSensorPosition();
  }

  @Override
  public void setRotation(boolean inverted) { // in Ticks
    wristMotor.set(TalonSRXControlMode.MotionMagic, inverted ? 3655 : 1607);
  }

  @Override
  public double getDegrees() {
    return wristMotor.getSelectedSensorPosition();
  }

  @Override
  public double getCurrentAmps() {
    return wristMotor.getStatorCurrent();
  }
}
