package frc.robot.subsystems;

import static frc.robot.Constants.TAB_WRIST;
import static frc.robot.Constants.WRIST_OFFSET;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetWristPosition;

public class Wrist extends SubsystemBase {

  private final WPI_TalonSRX wristMotor;
  private final double currentThreshold = 10.0; // amps
  public static final int ANGLE_STRAIGHT = 1457 + WRIST_OFFSET; // 2062
  public static final int ANGLE_FRONT_MAX = 795; // when telescope extended
  public static final int ANGLE_FRONT_PERPENDICULAR = 447;
  public static final int ANGLE_BACK_PERPENDICULAR = 2439;
  public static final int ANGLE_BACK_MAX = 3393; // when telescope extended
  public static final int ANGLE_BACK_HALF = 2635; // when telescope extended
  public static final int ANGLE_FRONT_HALF = 1924; // when telescope extended

  /** Creates a new Wrist. */
  public Wrist(int wristMotorID) {
    wristMotor = new WPI_TalonSRX(wristMotorID);
    TalonSRXConfiguration wristConfig = new TalonSRXConfiguration();
    wristConfig.motionAcceleration = 600;
    wristConfig.motionCruiseVelocity = 950;
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
    setPosition(ANGLE_STRAIGHT);
    TAB_WRIST.add(
        "reset",
        new InstantCommand(
            () -> {
              wristMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
              wristMotor.setSelectedSensorPosition(0);
            }));
    TAB_WRIST.addNumber("Current Position", wristMotor::getSelectedSensorPosition);
    TAB_WRIST.addNumber("Motor Voltage", wristMotor::getMotorOutputVoltage);
    int row = 0;
    TAB_WRIST.add("Set Normal", new InstantCommand(() -> setRotation(true))).withSize(2, 1);
    TAB_WRIST.add("Set Invert", new InstantCommand(() -> setRotation(false))).withSize(2, 1);

    TAB_WRIST.add("Set Straight", new SetWristPosition(ANGLE_STRAIGHT, this)).withSize(2, 1);
    TAB_WRIST
        .add("Set Front Perpendicular", new SetWristPosition(ANGLE_FRONT_PERPENDICULAR, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Back Perpendicular", new SetWristPosition(ANGLE_BACK_PERPENDICULAR, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Back Half Perpendicular", new SetWristPosition(ANGLE_BACK_HALF, this))
        .withSize(2, 1);
    TAB_WRIST
        .add("Set Front Half Perpendicular", new SetWristPosition(ANGLE_FRONT_HALF, this))
        .withSize(2, 1);
  }

  @Override
  public void periodic() {}

  public void setPosition(final int ticks) {
    wristMotor.set(TalonSRXControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, -0.1);
  }

  public int getPosition() {
    return (int) wristMotor.getSelectedSensorPosition();
  }

  public void setRotation(boolean inverted) { // in Ticks
    wristMotor.set(TalonSRXControlMode.MotionMagic, inverted ? 3655 : 1607);
  }

  public double getDegrees() {
    return wristMotor.getSelectedSensorPosition();
  }

  public double getCurrentAmps() {
    return wristMotor.getStatorCurrent();
  }
}
