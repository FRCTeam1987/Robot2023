package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.TunableNumber;

public class WristIOTalonSRX implements WristIO {

  private final WPI_TalonSRX wristMotor;

  public WristIOTalonSRX(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.feedbackNotContinuous = true;
    wristConfig.slot0.kP = 1.3;
    wristConfig.slot0.kD = 0.0;
    wristConfig.slot0.allowableClosedloopError = 0;
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(wristConfig);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    wristMotor.setSelectedSensorPosition(0);
    wristMotor.setSensorPhase(true);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(5);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.configClosedloopRamp(0.15);
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
    // wristMotor.set(TalonSRXControlMode.PercentOutput, SmartDashboard.getNumber("speed", 0.09));
    wristMotor.set(
        TalonSRXControlMode.Position, inverted ? 0 : 2048);
  }

  @Override
  public double getDegrees() {
    return (double) wristMotor.getSelectedSensorPosition();
    //  / 4096.0 * 360.0   /
  }
}
