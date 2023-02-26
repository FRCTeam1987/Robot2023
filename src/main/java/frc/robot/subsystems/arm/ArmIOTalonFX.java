package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;
import java.util.Map;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX ROTATION_LEADER_TALON;
  private final TalonFX ROTATION_FOLLOWER_TALON;
  private final CANCoder ROTATION_CANCODER;
  private final TalonFX EXTENSION_TALON;
  private final AnalogInput EXTENSION_POTENTIOMETER;
  private final Alert armOverRotateAlert =
      new Alert("Attempted to set arm rotation beyond safe range.", Alert.AlertType.ERROR);
  private final Alert armOverExtendAlert =
      new Alert("Attempted to set arm length beyond safe range.", Alert.AlertType.ERROR);

  public ArmIOTalonFX(
      final int ROTATION_LEADER_MOTOR_ID,
      final int ROTATION_FOLLOWER_MOTOR_ID,
      final int ROTATION_CANCODER_ID,
      final int EXTENSION_MOTOR_ID,
      final int ROTATION_ANALOG_POTENTIOMETER_ID,
      final String CAN_BUS_NAME) {

    ROTATION_CANCODER = new CANCoder(ROTATION_CANCODER_ID, CAN_BUS_NAME);
    ROTATION_CANCODER.configFactoryDefault();
    ROTATION_CANCODER.configAllSettings(getCANCoderConfig());
    ROTATION_CANCODER.setPosition(ROTATION_CANCODER.getAbsolutePosition() - CANCODER_OFFSET);

    ROTATION_LEADER_TALON = new TalonFX(ROTATION_LEADER_MOTOR_ID, CAN_BUS_NAME);
    ROTATION_LEADER_TALON.configFactoryDefault();
    ROTATION_LEADER_TALON.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    ROTATION_LEADER_TALON.configVoltageCompSaturation(ROTATION_VOLTAGE_COMPENSATION_SATURATION);
    ROTATION_LEADER_TALON.configClosedloopRamp(ROTATION_CLOSED_LOOP_RAMP_SECONDS);
    ROTATION_LEADER_TALON.configAllSettings(getRotMotorConfig(ROTATION_CANCODER_ID));
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
    EXTENSION_TALON.configAllSettings(getExtMotorConfig());
    EXTENSION_TALON.enableVoltageCompensation(true);
    EXTENSION_TALON.setNeutralMode(NeutralMode.Brake);
    EXTENSION_TALON.configGetStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, 25, 25, 150));

    EXTENSION_TALON.setSelectedSensorPosition(0.0);
    // use convertVoltsToInches of the pot

    EXTENSION_POTENTIOMETER = new AnalogInput(ROTATION_ANALOG_POTENTIOMETER_ID);

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

    return rotMotorConfig;
  }

  private TalonFXConfiguration getExtMotorConfig() {
    TalonFXConfiguration extConfig = new TalonFXConfiguration();
    extConfig.feedbackNotContinuous = true;

    extConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    extConfig.motionAcceleration = EXTENSION_MOTION_ACCELERATION;
    extConfig.motionCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extConfig.slot0.kP = EXTENSION_KP;
    extConfig.slot0.kI = EXTENSION_KI;
    extConfig.slot0.kD = EXTENSION_KD;
    extConfig.slot0.kF = 0.0;
    extConfig.slot0.allowableClosedloopError = EXTENSION_ALLOWABLE_ERROR;

    return extConfig;
  }

  private void setShuffleboardLayout() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm Tab");

    ShuffleboardLayout rotList =
        armTab.getLayout("Arm Rotation", BuiltInLayouts.kList).withSize(2, 4);

    GenericEntry targetAngle =
        rotList
            .add("Target Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -MAX_ROTATION_ANGLE, "max", MAX_ROTATION_ANGLE))
            .getEntry();
    rotList.add("Rotate Arm", new InstantCommand(() -> setArmAngle(targetAngle.getDouble(0))));
    rotList.addNumber("Arm Angle", this::getArmAngle);
    rotList.addNumber("Rotation Motor Ticks", (() -> convertDegreesToTicks(getArmAngle())));

    ShuffleboardLayout extList =
        armTab.getLayout("Arm Extension", BuiltInLayouts.kList).withSize(2, 4);

    GenericEntry targetLength = armTab.add("Target Length", 0).getEntry();
    armTab.add("Extend Arm", new InstantCommand(() -> setArmLength(targetLength.getDouble(0))));

    extList.addNumber("Arm Length Inches", this::getArmLength);
    extList.addNumber("Extension Motor Ticks", EXTENSION_TALON::getSelectedSensorPosition);
    extList.addNumber("Potentiometer Voltage", EXTENSION_POTENTIOMETER::getVoltage);

    armTab.add(
        "Coast Extension Motors",
        new InstantCommand(() -> EXTENSION_TALON.setNeutralMode(NeutralMode.Coast))
            .ignoringDisable(true));
    armTab.add(
        "Brake Extension Motors",
        new InstantCommand(() -> EXTENSION_TALON.setNeutralMode(NeutralMode.Brake))
            .ignoringDisable(true));
  }

  public int convertDegreesToTicks(double degrees) {
    return (int) (degrees * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  public double convertTicksToDegrees(double ticks) {
    return (ticks * CONVERSION_FACTOR_DEGREES_TO_TICKS);
  }

  private int convertInchesToTicks(double inches) {
    System.out.println(inches + " " + inches * CONVERSION_FACTOR_INCHES_TO_TICKS);
    return (int) (inches * CONVERSION_FACTOR_INCHES_TO_TICKS);
  }

  private int convertTicksToInches(double inches) {
    return (int) (inches * CONVERSION_FACTOR_TICKS_TO_INCHES);
  }

  private double convertVoltsToInches(double volts) {
    return (volts * CONVERSION_FACTOR_VOLTAGE_TO_INCHES);
  }

  @Override
  public double getArmLength() {
    return (convertTicksToInches(
        EXTENSION_TALON.getSelectedSensorPosition() - MINIMUM_EXTENSION_MOTOR_TICKS));
  }

  @Override
  public void setArmLength(double inches) {
    if (MINIMUM_EXTENSION_LENGTH_INCHES < inches && inches < MAXIMUM_EXTENSION_LENGTH_INCHES) {
      EXTENSION_TALON.set(TalonFXControlMode.MotionMagic, convertInchesToTicks(inches));
    } else {
      armOverExtendAlert.set(true);
    }
  }

  @Override
  public double getArmAngle() {
    return ROTATION_CANCODER.getPosition();
  }

  @Override
  public void setArmAngle(double angle) {
    if (Math.abs(angle) < MAX_ROTATION_ANGLE) {
      ROTATION_LEADER_TALON.set(TalonFXControlMode.MotionMagic, convertDegreesToTicks(angle));
    } else {
      armOverRotateAlert.set(true);
    }
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps =
        new double[] {
          ROTATION_LEADER_TALON.getStatorCurrent(),
          ROTATION_FOLLOWER_TALON.getStatorCurrent(),
          EXTENSION_TALON.getStatorCurrent()
        };
    inputs.currentVolts =
        new double[] {
          ROTATION_LEADER_TALON.getMotorOutputVoltage(),
          ROTATION_FOLLOWER_TALON.getMotorOutputVoltage(),
          EXTENSION_TALON.getMotorOutputVoltage()
        };
    inputs.armAbsoluteAngle = getArmAngle();
  }
}
