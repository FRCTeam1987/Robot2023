package frc.robot.subsystems.arm;

import edu.wpi.first.util.InterpolatingTreeMap;

public final class ArmConstants {

  public static final double ROTATION_KP = 2.75; // 2.75
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 1.5;
  public static final double ROTATION_KF = -0.04392085;
  public static final double ROTATION_MOTION_ACCELERATION = 1750;
  public static final double ROTATION_CRUISE_VELOCITY = 2250;
  public static final double ROTATION_ALLOWABLE_ERROR = 0.0; // 20

  public static final double EXTENSION_KP = 1.6;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.0;
  public static final double EXTENSION_MOTION_ACCELERATION = 50000;
  public static final double EXTENSION_CRUISE_VELOCITY = 65000;
  public static final double EXTENSION_ALLOWABLE_ERROR = 500.0;

  public static final double CANCODER_OFFSET = -261.475;//-321.475
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

  public static final double ROTATION_VOLTAGE_COMPENSATION_SATURATION = 8;
  public static final double EXTENSION_VOLTAGE_COMPENSATION_SATURATION = 10;
  public static final double ROTATION_CLOSED_LOOP_RAMP_SECONDS = 0.3;
  public static final double EXTENSION_CLOSED_LOOP_RAMP_SECONDS = 0.1;

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
}
