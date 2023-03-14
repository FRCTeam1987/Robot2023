package frc.robot.subsystems;

import static frc.robot.Constants.TAB_ARM;
import static frc.robot.subsystems.constants.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ArmConstants;
import frc.robot.util.util6328.Alert;
import java.util.Map;

public class Arm extends SubsystemBase {
  private final TalonFX ROTATION_LEADER_TALON;
  private final TalonFX ROTATION_FOLLOWER_TALON;
  private final CANCoder ROTATION_CANCODER;
  private final TalonFX EXTENSION_TALON;
  private final AnalogInput EXTENSION_POTENTIOMETER;
  private final Alert armOverRotateAlert =
      new Alert("Attempted to set arm rotation beyond safe range.", Alert.AlertType.ERROR);
  private final Alert armOverExtendAlert =
      new Alert("Attempted to set arm length beyond safe range.", Alert.AlertType.ERROR);

  public static final int HOME_ROTATION = 0;
  public static final int HOME_EXTENSION = 1;

  /**
   * @param ROTATION_LEADER_MOTOR_ID
   * @param ROTATION_FOLLOWER_MOTOR_ID
   * @param ROTATION_CANCODER_ID
   * @param EXTENSION_MOTOR_ID
   * @param ROTATION_ANALOG_POTENTIOMETER_ID
   * @param CAN_BUS_NAME
   */
  public Arm(
      final int ROTATION_LEADER_MOTOR_ID,
      final int ROTATION_FOLLOWER_MOTOR_ID,
      final int ROTATION_CANCODER_ID,
      final int EXTENSION_MOTOR_ID,
      final int ROTATION_ANALOG_POTENTIOMETER_ID,
      final String CAN_BUS_NAME) {

    ROTATION_CANCODER = new CANCoder(ROTATION_CANCODER_ID, CAN_BUS_NAME);
    ROTATION_CANCODER.configFactoryDefault();
    ROTATION_CANCODER.configAllSettings(ArmConstants.getCANCoderConfig());
    ROTATION_CANCODER.setPosition(ROTATION_CANCODER.getAbsolutePosition() + CANCODER_OFFSET);

    ROTATION_LEADER_TALON = new TalonFX(ROTATION_LEADER_MOTOR_ID, CAN_BUS_NAME);
    ROTATION_LEADER_TALON.configFactoryDefault();
    ROTATION_LEADER_TALON.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    ROTATION_LEADER_TALON.configVoltageCompSaturation(ROTATION_VOLTAGE_COMPENSATION_SATURATION);
    ROTATION_LEADER_TALON.configClosedloopRamp(ROTATION_CLOSED_LOOP_RAMP_SECONDS);
    ROTATION_LEADER_TALON.configAllSettings(ArmConstants.getRotMotorConfig(ROTATION_CANCODER_ID));
    ROTATION_LEADER_TALON.setNeutralMode(NeutralMode.Brake);
    ROTATION_LEADER_TALON.enableVoltageCompensation(true);

    ROTATION_FOLLOWER_TALON = new TalonFX(ROTATION_FOLLOWER_MOTOR_ID, CAN_BUS_NAME);
    ROTATION_FOLLOWER_TALON.configFactoryDefault();
    ROTATION_FOLLOWER_TALON.configVoltageCompSaturation(ROTATION_VOLTAGE_COMPENSATION_SATURATION);
    ROTATION_FOLLOWER_TALON.follow(ROTATION_LEADER_TALON);
    ROTATION_FOLLOWER_TALON.setNeutralMode(NeutralMode.Brake);
    ROTATION_FOLLOWER_TALON.enableVoltageCompensation(true);

    EXTENSION_TALON = new TalonFX(EXTENSION_MOTOR_ID);
    EXTENSION_TALON.configFactoryDefault();
    EXTENSION_TALON.configVoltageCompSaturation(EXTENSION_VOLTAGE_COMPENSATION_SATURATION);
    EXTENSION_TALON.configClosedloopRamp(EXTENSION_CLOSED_LOOP_RAMP_SECONDS);
    EXTENSION_TALON.configAllSettings(ArmConstants.getExtMotorConfig());
    EXTENSION_TALON.enableVoltageCompensation(true);
    EXTENSION_TALON.setNeutralMode(NeutralMode.Brake);
    EXTENSION_TALON.configGetStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, 25, 25, 150));
    EXTENSION_TALON.setSelectedSensorPosition(0.0);

    EXTENSION_POTENTIOMETER = new AnalogInput(ROTATION_ANALOG_POTENTIOMETER_ID);
    // EXTENSION_TALON.setSelectedSensorPosition(convertVoltsToInches(EXTENSION_POTENTIOMETER.getVoltage()))

    // ArmFeedforward ff = new ArmFeedforward(0, 0.38, 2.23);
    // ff.calculate(EXTENSION_MOTOR_ID, ROTATION_ANALOG_POTENTIOMETER_ID);
    // ROTATION_LEADER_TALON.configNominalOutputForward(ROTATION_ANALOG_POTENTIOMETER_ID)

    setShuffleboardLayout();
  }

  @Override
  public void periodic() {}

  public int convertDegreesToTicks(double degrees) {
    return (int) (degrees * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  public double convertTicksToDegrees(double ticks) {
    return (ticks * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  public void coastArmRotation() {
    ROTATION_LEADER_TALON.setNeutralMode(NeutralMode.Coast);
    ROTATION_FOLLOWER_TALON.setNeutralMode(NeutralMode.Coast);
  }

  public void coastArmExtension() {
    EXTENSION_TALON.setNeutralMode(NeutralMode.Coast);
  }

  public void brakeArmRotation() {
    ROTATION_LEADER_TALON.setNeutralMode(NeutralMode.Brake);
    ROTATION_FOLLOWER_TALON.setNeutralMode(NeutralMode.Brake);
  }

  public void brakeArmExtension() {
    EXTENSION_TALON.setNeutralMode(NeutralMode.Brake);
  }

  private int convertInchesToTicks(double inches) {
    return (int) (inches * CONVERSION_FACTOR_INCHES_TO_TICKS);
  }

  private double convertTicksToInches(double ticks) {
    return ticks * CONVERSION_FACTOR_TICKS_TO_INCHES;
  }

  private double convertVoltsToInches(double volts) {
    return (volts * CONVERSION_FACTOR_VOLTAGE_TO_INCHES);
  }

  public double getArmLength() {
    return convertTicksToInches(
        EXTENSION_TALON.getSelectedSensorPosition() - MINIMUM_EXTENSION_MOTOR_TICKS);
  }

  public void setArmLength(double inches) {
    if (MINIMUM_EXTENSION_LENGTH_INCHES < inches && inches < MAXIMUM_EXTENSION_LENGTH_INCHES) {
      EXTENSION_TALON.set(
          TalonFXControlMode.MotionMagic,
          convertInchesToTicks(inches),
          DemandType.ArbitraryFeedForward,
          0.095 * Math.sin(Math.toRadians(90.0 - getArmAngle())));
    } else {
      armOverExtendAlert.set(true);
    }
  }

  public double getArmAngle() {
    return ROTATION_CANCODER.getPosition();
  }

  public void setArmAngle(double angle) {
    if (Math.abs(angle) < MAX_ROTATION_ANGLE) {
      // ROTATION_LEADER_TALON.config_kP(0, ROTATION_KP);
      ROTATION_LEADER_TALON.config_kP(0, 2.75);
      // ROTATION_LEADER_TALON.config_kP(0, 2.75);
      // ROTATION_LEADER_TALON.config_kD(0, ROTATION_KD);
      ROTATION_LEADER_TALON.config_kD(0, 0);
      ROTATION_LEADER_TALON.set(TalonFXControlMode.MotionMagic, convertDegreesToTicks(angle));
      // DemandType.ArbitraryFeedForward,
      // (ArmConstants.rotationArbitraryFeedforwardValues.get(getArmLength())) *
      // Math.cos(Math.toRadians(90.0 - angle))); //
      // // ROTATION_KF * Math.cos(Math.toRadians(90.0 - angle))); //
    } else {
      armOverRotateAlert.set(true);
    }
  }

  public void stallArm() {
    ROTATION_LEADER_TALON.set(
        TalonFXControlMode.PercentOutput,
        ArmConstants.rotationArbitraryFeedforwardValues.get(getArmLength())
            * Math.cos(Math.toRadians(90.0 - this.getArmAngle())));
    // (-0.095) * Math.cos(Math.toRadians(90.0 - this.getArmAngle())));
  }

  public void holdCurrentAngle() {
    double angle = this.getArmAngle();
    if (Math.abs(angle) < MAX_ROTATION_ANGLE) {
      ROTATION_LEADER_TALON.config_kP(0, 3.25);
      ROTATION_LEADER_TALON.set(
          TalonFXControlMode.Position,
          convertDegreesToTicks(angle),
          DemandType.ArbitraryFeedForward,
          (-0.095) * Math.cos(Math.toRadians(90.0 - this.getArmAngle())));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  public void holdCurrentAngle(double desiredPosition) {
    if (Math.abs(desiredPosition) < MAX_ROTATION_ANGLE) {
      ROTATION_LEADER_TALON.config_kP(0, 0.1);
      // ROTATION_LEADER_TALON.config_kP(0, 0.4);
      // ROTATION_LEADER_TALON.config_kP(0, 3.25);
      ROTATION_LEADER_TALON.config_kD(0, 0.0);
      // ROTATION_LEADER_TALON.config_kD(0, 0.003);
      ROTATION_LEADER_TALON.set(
          TalonFXControlMode.Position,
          convertDegreesToTicks(desiredPosition),
          DemandType.ArbitraryFeedForward,
          // (-0.05675) * Math.cos(Math.toRadians(90.0 - desiredPosition)));
          (-0.055) * Math.cos(Math.toRadians(90.0 - desiredPosition)));
      // (-0.095) * Math.cos(Math.toRadians(90.0 - desiredPosition)));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  public void setExtensionNominal() { // give the extension motor a break when driving around
    EXTENSION_TALON.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void stop() {
    EXTENSION_TALON.set(ControlMode.PercentOutput, 0);
    ROTATION_LEADER_TALON.set(ControlMode.PercentOutput, 0);
  }

  private void setShuffleboardLayout() {

    ShuffleboardLayout rotList =
        TAB_ARM.getLayout("Arm Rotation", BuiltInLayouts.kList).withSize(1, 5);

    GenericEntry targetAngle =
        rotList
            .add("Target Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -MAX_ROTATION_ANGLE, "max", MAX_ROTATION_ANGLE))
            .getEntry();
    TAB_ARM.add("Rotate Arm", new InstantCommand(() -> setArmAngle(targetAngle.getDouble(0))));
    rotList.addNumber("Arm Angle", this::getArmAngle);
    rotList.addNumber("Rotation Motor Ticks", (() -> convertDegreesToTicks(getArmAngle())));

    rotList.addNumber("Rotation voltage", ROTATION_LEADER_TALON::getMotorOutputVoltage);
    rotList.addNumber("Rotation current", ROTATION_LEADER_TALON::getSupplyCurrent);
    rotList.addNumber("Rotation error", ROTATION_LEADER_TALON::getClosedLoopError);
    TAB_ARM.addNumber("SensorSelectedVelocity", ROTATION_LEADER_TALON::getSelectedSensorVelocity);

    ShuffleboardLayout extList =
        TAB_ARM.getLayout("Arm Extension", BuiltInLayouts.kList).withSize(1, 4);

    GenericEntry targetLength = extList.add("Target Length", 0).getEntry();

    extList.addNumber("Arm Length Inches", this::getArmLength);
    extList.addNumber("Extension Motor Ticks", EXTENSION_TALON::getSelectedSensorPosition);
    extList.addNumber("Potentiometer Voltage", EXTENSION_POTENTIOMETER::getVoltage);

    TAB_ARM.add("Extend Arm", new InstantCommand(() -> setArmLength(targetLength.getDouble(0))));

    TAB_ARM.add("Coast Rotation", new InstantCommand(this::coastArmRotation).ignoringDisable(true));
    TAB_ARM.add(
        "Coast Extension", new InstantCommand(this::coastArmExtension).ignoringDisable(true));
    TAB_ARM.add(
        "Brake Extension", new InstantCommand(this::brakeArmExtension).ignoringDisable(true));
    TAB_ARM.add("Brake Rotation", new InstantCommand(this::brakeArmRotation).ignoringDisable(true));
    TAB_ARM.addNumber("Arm Angle", this::getArmAngle);
    TAB_ARM.addNumber("Arm Length", this::getArmLength);
  }
}
