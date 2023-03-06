package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WristIOTalonSRX implements WristIO {

  public static final ShuffleboardTab tab = Shuffleboard.getTab("wrist");
  public static final int ANGLE_STRAIGHT = 2289;

  private final WPI_TalonSRX wristMotor;

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.motionAcceleration = 600;
    wristConfig.motionCruiseVelocity = 950;
    wristConfig.feedbackNotContinuous = true;
    wristConfig.slot0.kP = 3.75;
    wristConfig.slot0.kD = 0.0;
    wristConfig.slot0.allowableClosedloopError = 0;
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(wristConfig);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(8);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setInverted(InvertType.InvertMotorOutput);
    wristMotor.setSensorPhase(true);
    wristMotor.configContinuousCurrentLimit(25);
    wristMotor.configPeakCurrentLimit(30);
    setPosition(ANGLE_STRAIGHT);
    tab.add(
        "reset",
        new InstantCommand(
            () -> {
              wristMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
              wristMotor.setSelectedSensorPosition(0);
            }));
    tab.addNumber(
        "Current Position",
        () -> {
          return wristMotor.getSelectedSensorPosition();
        });
    tab.addNumber(
        "motor voltage",
        () -> {
          return wristMotor.getMotorOutputVoltage();
        });
    SmartDashboard.putNumber("speed", 0.0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = wristMotor.getStatorCurrent();
    inputs.targetPositionRotations = wristMotor.getSelectedSensorPosition();
  }

  public void setPosition(final int ticks) {
    wristMotor.set(TalonSRXControlMode.MotionMagic, ticks);
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
    return (double) wristMotor.getSelectedSensorPosition();
  }
}
