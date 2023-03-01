package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WristIOTalonSRX implements WristIO {

  private final WPI_TalonSRX wristMotor;

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.motionAcceleration = 750;
    wristConfig.motionCruiseVelocity = 1250;
    wristConfig.feedbackNotContinuous = true;
    wristConfig.slot0.kP = 1.0;
    wristConfig.slot0.kD = 0.0;
    wristConfig.slot0.allowableClosedloopError = 0;
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(wristConfig);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(7);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setInverted(InvertType.InvertMotorOutput);
    wristMotor.setSensorPhase(true);
    wristMotor.configContinuousCurrentLimit(17);
    wristMotor.configPeakCurrentLimit(25);
    Shuffleboard.getTab("wrist")
        .add(
            "reset",
            new InstantCommand(
                () -> {
                  wristMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
                  wristMotor.setSelectedSensorPosition(0);
                }));
    SmartDashboard.putNumber("speed", 0.0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentAmps = wristMotor.getStatorCurrent();
    inputs.targetPositionRotations = wristMotor.getSelectedSensorPosition();
  }

  @Override
  public void setPosition(boolean inverted) { // in Ticks
    wristMotor.set(TalonSRXControlMode.MotionMagic, inverted ? 3655 : 1607);
  }

  @Override
  public double getDegrees() {
    return wristMotor.getSelectedSensorPosition();
  }
}
