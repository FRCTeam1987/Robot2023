package frc.robot.subsystems.arm;

import static frc.robot.Constants.TAB_MAIN;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX rotationLeaderTalon;
  private final TalonFX roationFollowerTalon;
  private final CANCoder rotationCANCoder;
  private final TalonFX extensionTalon;
  private final Alert armOverRotateAlert =
      new Alert("Attempted to set arm rotation beyond safe range.", Alert.AlertType.ERROR);
  private final Alert armOverExtendAlert =
      new Alert("Attempted to set arm length beyond safe range.", Alert.AlertType.ERROR);

  public ArmIOTalonFX(
      final int ROTATION_LEADER_MOTOR_ID,
      final int ROTATION_FOLLOWER_MOTOR_ID,
      final int ROTATION_CANCODER_ID,
      final int EXTENSION_MOTOR_ID,
      final String CAN_BUS_NAME) {

    rotationCANCoder = new CANCoder(ROTATION_CANCODER_ID, CAN_BUS_NAME);
    rotationCANCoder.configFactoryDefault();
    rotationCANCoder.configAllSettings(getCANCoderConfig());
    rotationCANCoder.setPosition(
        rotationCANCoder.getAbsolutePosition() + Constants.INSTALLED_ARM.getCancoderOffset());

    rotationLeaderTalon = new TalonFX(ROTATION_LEADER_MOTOR_ID, CAN_BUS_NAME);
    rotationLeaderTalon.configFactoryDefault();
    rotationLeaderTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    rotationLeaderTalon.configVoltageCompSaturation(ROTATION_VOLTAGE_COMPENSATION_SATURATION);
    rotationLeaderTalon.configClosedloopRamp(ROTATION_CLOSED_LOOP_RAMP_SECONDS);
    rotationLeaderTalon.configAllSettings(getRotMotorConfig(ROTATION_CANCODER_ID));
    rotationLeaderTalon.setNeutralMode(NeutralMode.Brake);
    rotationLeaderTalon.enableVoltageCompensation(true);

    roationFollowerTalon = new TalonFX(ROTATION_FOLLOWER_MOTOR_ID, CAN_BUS_NAME);
    roationFollowerTalon.configFactoryDefault();
    roationFollowerTalon.configVoltageCompSaturation(ROTATION_VOLTAGE_COMPENSATION_SATURATION);
    roationFollowerTalon.follow(rotationLeaderTalon);
    roationFollowerTalon.setNeutralMode(NeutralMode.Brake);
    roationFollowerTalon.enableVoltageCompensation(true);

    extensionTalon = new TalonFX(EXTENSION_MOTOR_ID);
    extensionTalon.configFactoryDefault();
    extensionTalon.configVoltageCompSaturation(EXTENSION_VOLTAGE_COMPENSATION_SATURATION);
    extensionTalon.configClosedloopRamp(EXTENSION_CLOSED_LOOP_RAMP_SECONDS);
    extensionTalon.configAllSettings(getExtMotorConfig());
    extensionTalon.enableVoltageCompensation(true);
    extensionTalon.setNeutralMode(NeutralMode.Brake);
    extensionTalon.configGetStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, 25, 25, 150));
    extensionTalon.setSelectedSensorPosition(0.0);

    setShuffleboardLayout();
  }

  private CANCoderConfiguration getCANCoderConfig() {
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    canCoderConfig.sensorDirection = false;

    return canCoderConfig;
  }

  private TalonFXConfiguration getRotMotorConfig(int rotCANCoderID) {
    TalonFXConfiguration rotMotorConfig = new TalonFXConfiguration();
    rotMotorConfig.remoteFilter0.remoteSensorDeviceID = rotCANCoderID;
    rotMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    rotMotorConfig.feedbackNotContinuous = true;

    rotMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    rotMotorConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    rotMotorConfig.motionAcceleration = ROTATION_MOTION_ACCELERATION;
    rotMotorConfig.motionCruiseVelocity = ROTATION_CRUISE_VELOCITY;
    rotMotorConfig.slot0.kP = ROTATION_KP;
    rotMotorConfig.slot0.kI = ROTATION_KI;
    rotMotorConfig.slot0.kD = ROTATION_KD;
    rotMotorConfig.slot0.kF = 0.0;
    rotMotorConfig.slot0.allowableClosedloopError = ROTATION_ALLOWABLE_ERROR;
    rotMotorConfig.neutralDeadband = 0.001;

    return rotMotorConfig;
  }

  private TalonFXConfiguration getExtMotorConfig() {
    TalonFXConfiguration extConfig = new TalonFXConfiguration();
    extConfig.feedbackNotContinuous = true;

    extConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    extConfig.motionAcceleration = EXTENSION_MOTION_ACCELERATION;
    extConfig.motionCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extConfig.slot0.kP = 0.1;
    extConfig.slot0.kI = EXTENSION_KI;
    extConfig.slot0.kD = EXTENSION_KD;
    extConfig.slot0.kF = 0.0;
    extConfig.slot0.allowableClosedloopError = EXTENSION_ALLOWABLE_ERROR;
    extConfig.neutralDeadband = 0.001;

    return extConfig;
  }

  private void setShuffleboardLayout() {

    TAB_MAIN
        .add("Coast Rotation", new InstantCommand(this::coastArmRotation).ignoringDisable(true))
        .withPosition(4, 0);
    TAB_MAIN
        .add("Coast Extension", new InstantCommand(this::coastArmExtension).ignoringDisable(true))
        .withPosition(5, 0);
    TAB_MAIN
        .add("Brake Extension", new InstantCommand(this::brakeArmExtension).ignoringDisable(true))
        .withPosition(5, 1);
    TAB_MAIN
        .add("Brake Rotation", new InstantCommand(this::brakeArmRotation).ignoringDisable(true))
        .withPosition(4, 1);
  }

  public int convertDegreesToTicks(double degrees) {
    return (int) (degrees * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  public double convertTicksToDegrees(double ticks) {
    return (ticks * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  public void coastArmRotation() {
    rotationLeaderTalon.setNeutralMode(NeutralMode.Coast);
    roationFollowerTalon.setNeutralMode(NeutralMode.Coast);
  }

  public void coastArmExtension() {
    extensionTalon.setNeutralMode(NeutralMode.Coast);
  }

  public void brakeArmRotation() {
    rotationLeaderTalon.setNeutralMode(NeutralMode.Brake);
    roationFollowerTalon.setNeutralMode(NeutralMode.Brake);
  }

  public void brakeArmExtension() {
    extensionTalon.setNeutralMode(NeutralMode.Brake);
  }

  private int convertInchesToTicks(double inches) {
    return (int) (inches * CONVERSION_FACTOR_INCHES_TO_TICKS);
  }

  private double convertTicksToInches(double ticks) {
    return ticks * CONVERSION_FACTOR_TICKS_TO_INCHES;
  }

  @Override
  public double getArmLength() {
    return convertTicksToInches(
        extensionTalon.getSelectedSensorPosition() - MINIMUM_EXTENSION_MOTOR_TICKS);
  }

  @Override
  public void setArmLength(double inches) {
    if (MINIMUM_EXTENSION_LENGTH_INCHES < inches && inches < MAXIMUM_EXTENSION_LENGTH_INCHES) {
      extensionTalon.set(
          TalonFXControlMode.MotionMagic,
          convertInchesToTicks(inches),
          DemandType.ArbitraryFeedForward,
          0.095 * Math.sin(Math.toRadians(90.0 - getArmAngle())));
    } else {
      armOverExtendAlert.set(true);
    }
  }

  @Override
  public double getArmAngle() {
    return rotationCANCoder.getPosition();
  }

  @Override
  public void setArmAngle(double angle) {
    if (Math.abs(angle) < MAX_ROTATION_ANGLE) {
      rotationLeaderTalon.config_kP(0, 2.75);
      rotationLeaderTalon.config_kD(0, 0);
      rotationLeaderTalon.set(TalonFXControlMode.MotionMagic, convertDegreesToTicks(angle));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  @Override
  public void stallArm() {
    rotationLeaderTalon.set(
        TalonFXControlMode.PercentOutput,
        ArmConstants.rotationArbitraryFeedforwardValues.get(getArmLength())
            * Math.cos(Math.toRadians(90.0 - this.getArmAngle())));
  }

  @Override
  public void holdCurrentAngle() {
    double angle = this.getArmAngle();
    if (Math.abs(angle) < MAX_ROTATION_ANGLE) {
      rotationLeaderTalon.config_kP(0, 3.25);
      rotationLeaderTalon.set(
          TalonFXControlMode.Position,
          convertDegreesToTicks(angle),
          DemandType.ArbitraryFeedForward,
          (-0.095) * Math.cos(Math.toRadians(90.0 - this.getArmAngle())));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  @Override
  public void holdCurrentAngle(double desiredPosition) {
    if (Math.abs(desiredPosition) < MAX_ROTATION_ANGLE) {
      rotationLeaderTalon.config_kP(0, 0.1);
      rotationLeaderTalon.config_kD(0, 0.0);
      rotationLeaderTalon.set(
          TalonFXControlMode.Position,
          convertDegreesToTicks(desiredPosition),
          DemandType.ArbitraryFeedForward,
          (-0.055) * Math.cos(Math.toRadians(90.0 - desiredPosition)));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  @Override
  public void setExtensionNominal() { // give the extension motor a break when driving around
    extensionTalon.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void stop() {
    extensionTalon.set(ControlMode.PercentOutput, 0);
    rotationLeaderTalon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps =
        new double[] {
          rotationLeaderTalon.getStatorCurrent(),
          roationFollowerTalon.getStatorCurrent(),
          extensionTalon.getStatorCurrent()
        };
    inputs.currentVolts =
        new double[] {
          rotationLeaderTalon.getMotorOutputVoltage(),
          roationFollowerTalon.getMotorOutputVoltage(),
          extensionTalon.getMotorOutputVoltage()
        };
    inputs.armAbsoluteAngle = getArmAngle();
  }
}
