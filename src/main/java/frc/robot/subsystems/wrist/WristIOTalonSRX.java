package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class WristIOTalonSRX implements WristIO {
  public static final int ANGLE_STRAIGHT = 1279 + Constants.INSTALLED_ARM.getWristOffset();

  private final WPI_TalonSRX wristMotor;

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.motionAcceleration = 4000;
    wristConfig.motionCruiseVelocity = 6000;
    wristConfig.feedbackNotContinuous = true;
    wristConfig.slot0.kP = 8.0;
    wristConfig.slot0.kD = 0.0;
    wristConfig.slot0.allowableClosedloopError = 0;
    wristConfig.neutralDeadband = 0.001;
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(wristConfig);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(7);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setInverted(InvertType.InvertMotorOutput);
    wristMotor.setSensorPhase(true);
    wristMotor.configContinuousCurrentLimit(15);
    wristMotor.configPeakCurrentLimit(30);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = wristMotor.getStatorCurrent();
    inputs.targetPositionRotations = wristMotor.getSelectedSensorPosition();
  }

  public void setPosition(final int ticks) {
    // DriverStation.reportWarning("====== Returned ticks!: " + (ticks +
    // Constants.INSTALLED_ARM.getMatchOffset()), false);
    wristMotor.set(
        TalonSRXControlMode.MotionMagic,
        ticks + Constants.INSTALLED_ARM.getMatchOffset(),
        DemandType.ArbitraryFeedForward,
        -0.1);
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

  @Override
  public void configRelative(final int homeTicks) {
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    wristMotor.setSelectedSensorPosition(homeTicks);
  }

  @Override
  public void setPercent(final double percent) {
    wristMotor.set(ControlMode.PercentOutput, percent);
  }
}
