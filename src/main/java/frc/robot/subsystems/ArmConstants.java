package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.util.InterpolatingTreeMap;

public final class ArmConstants {

  public static final double ROTATION_KP = 2.75; // 2.75
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 1.5;
  public static final double ROTATION_KF = -0.04392085;
  public static final double ROTATION_MOTION_ACCELERATION = 400;
  public static final double ROTATION_CRUISE_VELOCITY = 800;
  public static final double ROTATION_ALLOWABLE_ERROR = 0.0; // 20

  public static final double EXTENSION_KP = 1.6;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.0;
  public static final double EXTENSION_MOTION_ACCELERATION = 15000;
  public static final double EXTENSION_CRUISE_VELOCITY = 15000;
  public static final double EXTENSION_ALLOWABLE_ERROR = 500.0;

  public static final double CANCODER_OFFSET = -81.475;
  public static final double MAX_ROTATION_ANGLE = 115;
  public static final double FULL_ROTATION_DEGREES = 360.0;
  public static final double FULL_ROTATION_TICKS = 4096.0;
  public static final double MINIMUM_EXTENSION_LENGTH_INCHES = 0.0;
  public static final double MAXIMUM_EXTENSION_LENGTH_INCHES = 41.0;
  // FIXME: That's not supposed to be a thing! Make this value not 0. Original: -161.0
  public static final double MINIMUM_EXTENSION_MOTOR_TICKS = 123.0;
  public static final double MAXIMUM_EXTENSION_MOTOR_TICKS = 70500.0;
  public static final double MINIMUM_POTENTIOMETER_VOLTAGE = 0.0635;
  public static final double MAXIMUM_POTENTIOMETER_VOLTAGE = 3.497;
  public static final double CONVERSION_FACTOR_DEGREES_TO_TICKS =
      (FULL_ROTATION_TICKS) / (FULL_ROTATION_DEGREES);
  public static final double CONVERSION_FACTOR_TICKS_TO_DEGREES =
      (FULL_ROTATION_DEGREES) / (FULL_ROTATION_TICKS);
  public static final double CONVERSION_FACTOR_INCHES_TO_TICKS =
      (MAXIMUM_EXTENSION_MOTOR_TICKS - MINIMUM_EXTENSION_MOTOR_TICKS)
          / (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES);
  public static final double CONVERSION_FACTOR_TICKS_TO_INCHES =
      (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES)
          / (MAXIMUM_EXTENSION_MOTOR_TICKS - MINIMUM_EXTENSION_MOTOR_TICKS);
  public static final double CONVERSION_FACTOR_VOLTAGE_TO_INCHES =
      (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES)
          / (MAXIMUM_POTENTIOMETER_VOLTAGE - MINIMUM_POTENTIOMETER_VOLTAGE);

  public static final double ROTATION_VOLTAGE_COMPENSATION_SATURATION = 6;
  public static final double EXTENSION_VOLTAGE_COMPENSATION_SATURATION = 10;
  public static final double ROTATION_CLOSED_LOOP_RAMP_SECONDS = 0.5;
  public static final double EXTENSION_CLOSED_LOOP_RAMP_SECONDS = 0.25;

  public static final InterpolatingTreeMap<Double, Double> rotationArbitraryFeedforwardValues =
      new InterpolatingTreeMap<>();

  static {
    // rotationArbitraryFeedforwardValues.put(0.0, -0.09);
    // rotationArbitraryFeedforwardValues.put(1.0, -0.10);
    // rotationArbitraryFeedforwardValues.put(2.0, -0.11);
    rotationArbitraryFeedforwardValues.put(0.0, -0.07);
    rotationArbitraryFeedforwardValues.put(3.0, -0.072);
    rotationArbitraryFeedforwardValues.put(6.0, -0.074);
    rotationArbitraryFeedforwardValues.put(12.0, -0.0785);
    rotationArbitraryFeedforwardValues.put(17.0, -0.08);
    rotationArbitraryFeedforwardValues.put(21.0, -0.085);
    rotationArbitraryFeedforwardValues.put(24.0, -0.0875);
    rotationArbitraryFeedforwardValues.put(29.0, -0.091);
    rotationArbitraryFeedforwardValues.put(34.0, -0.095);
    rotationArbitraryFeedforwardValues.put(36.5, -0.0975);
    rotationArbitraryFeedforwardValues.put(39.5, -0.1);
  }

  public static CANCoderConfiguration getCANCoderConfig() {
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    canCoderConfig.sensorDirection = false;

    return canCoderConfig;
  }

  public static TalonFXConfiguration getRotMotorConfig(int rotCANCoderID) {
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

  public static TalonFXConfiguration getExtMotorConfig() {
    TalonFXConfiguration extConfig = new TalonFXConfiguration();
    extConfig.feedbackNotContinuous = true;

    extConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    extConfig.motionAcceleration = EXTENSION_MOTION_ACCELERATION;
    extConfig.motionCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extConfig.slot0.kP = 0.1;
    // extConfig.slot0.kP = EXTENSION_KP;
    extConfig.slot0.kI = EXTENSION_KI;
    extConfig.slot0.kD = EXTENSION_KD;
    extConfig.slot0.kF = 0.0;
    extConfig.slot0.allowableClosedloopError = EXTENSION_ALLOWABLE_ERROR;
    extConfig.neutralDeadband = 0.001;

    return extConfig;
  }
}
