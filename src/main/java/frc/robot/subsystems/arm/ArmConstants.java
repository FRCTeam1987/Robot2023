package frc.robot.subsystems.arm;

public final class ArmConstants {

  public static final double ROTATION_KP = 2.25;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 0.0;
  public static final double ROTATION_MOTION_ACCELERATION = 1000;
  public static final double ROTATION_CRUISE_VELOCITY = 1000;
  public static final double ROTATION_ALLOWABLE_ERROR = 0.0;

  public static final double EXTENSION_KP = 1.6;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.0;
  public static final double EXTENSION_MOTION_ACCELERATION = 15000;
  public static final double EXTENSION_CRUISE_VELOCITY = 15000;
  public static final double EXTENSION_ALLOWABLE_ERROR = 500.0;

  public static final double CANCODER_OFFSET = 200.918;
  public static final double MAX_ROTATION_ANGLE = 110;
  public static final double FULL_ROTATION_DEGREES = 360.0;
  public static final double FULL_ROTATION_TICKS = 4096.0;
  public static final double MINIMUM_EXTENSION_LENGTH_INCHES = 0.0;
  public static final double MAXIMUM_EXTENSION_LENGTH_INCHES = 41.0;
  // FIXME: That's not supposed to be a thing! Make this value not 0. Original: -161.0
  public static final double MINIMUM_EXTENSION_MOTOR_TICKS = 0;
  public static final double MAXIMUM_EXTENSION_MOTOR_TICKS = 71332.0;
  public static final double MINIMUM_POTENTIOMETER_VOLTAGE = 0.051269526;
  public static final double MAXIMUM_POTENTIOMETER_VOLTAGE = 4.44580032;
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
}
