package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.SwerveModuleConstants.SwerveType;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class CompRobotConfig extends RobotConfig {
  private static final int CLAW_MOTOR_ID = 8;

  public static final int WRIST_ROTATOR_MOTOR = 8;
  public static final double ROBOT_ARM_HEIGHT_OFFSET = 12.0;
  public static final int ARM_TELESCOPE_MOTOR = 6;
  public static final int ARM_CANCODER = 7;
  public static final int ARM_LEADER_MOTOR = 8;
  public static final int ARM_FOLLOWER_MOTOR = 9;
  public static final int ARM_POTENTIOMETER_ANALOG_ID = 3;
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 22;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 23;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 280.635;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 243.018;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 32;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 33;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 31;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = 9.316;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 42;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 43;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 41;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 63.633;
  private static final int GYRO_ID = 0;

  private static final double TRACKWIDTH_METERS = Units.inchesToMeters(18.75); // 22.5 inches
  private static final double WHEELBASE_METERS = Units.inchesToMeters(21.75); // 23.5 inches
  private static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 0.45;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 12.0;
  private static final double ANGLE_KF = 0.0;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 0.1;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KF = 0.0;

  private static final double DRIVE_KS = 0.55493;
  private static final double DRIVE_KV = 2.3014;
  private static final double DRIVE_KA = 0.12872;

  private static final SwerveType SWERVE_TYPE = SwerveType.MK4I;

  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.25;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  private static final String CAN_BUS_NAME = "canfd";

  private static final String CAMERA_NAME = "ov9268";

  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  private static final int PNEUMATICS_HUB_ID = 20;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  private static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 10.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  private static final int ANGLE_STRAIGHT = 2289;
  private static final int ANGLE_FRONT_MAX = 795; // when telescope extended
  private static final int ANGLE_FRONT_PERPENDICULAR = 1275;
  private static final int ANGLE_BACK_PERPENDICULAR = 3289;
  private static final int ANGLE_BACK_MAX = 3393; // when telescope extended
  private static final int ANGLE_BACK_HALF = 2635; // when telescope extended
  private static final int ANGLE_FRONT_HALF = 1924; // when telescope extended

  public int getAngleStraight() {
    return ANGLE_STRAIGHT;
  }

  public int getAngleFrontMax() {
    return ANGLE_FRONT_MAX;
  }

  public int getAngleFrontPerpendicular() {
    return ANGLE_FRONT_PERPENDICULAR;
  }

  public int getAngleBackPerpendicular() {
    return ANGLE_BACK_PERPENDICULAR;
  }

  public int getAngleBackMax() {
    return ANGLE_BACK_MAX;
  }

  public int getAngleBackHalf() {
    return ANGLE_BACK_HALF;
  }

  public int getAngleFrontHalf() {
    return ANGLE_FRONT_HALF;
  }

  public int getClawMotorID() {
    return CLAW_MOTOR_ID;
  }

  @Override
  public int getArmPotentiometerAnalogId() {
    return ARM_POTENTIOMETER_ANALOG_ID;
  }

  @Override
  public int getWristRotatorID() {
    return WRIST_ROTATOR_MOTOR;
  }

  public double getRobotArmHeightOffset() {
    return ROBOT_ARM_HEIGHT_OFFSET;
  }

  @Override
  public int getArmTelescopeID() {
    return ARM_TELESCOPE_MOTOR;
  }

  @Override
  public int getArmCanCoderID() {
    return ARM_CANCODER;
  }

  @Override
  public int getArmLeaderMotorID() {
    return ARM_LEADER_MOTOR;
  }

  @Override
  public int getArmFollowerMotorID() {
    return ARM_FOLLOWER_MOTOR;
  }

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveAngleKF() {
    return ANGLE_KF;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public double getSwerveDriveKF() {
    return DRIVE_KF;
  }

  @Override
  public double getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public double getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public double getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public SwerveType getSwerveType() {
    return SWERVE_TYPE;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {
      FRONT_LEFT_MODULE_STEER_OFFSET,
      FRONT_RIGHT_MODULE_STEER_OFFSET,
      BACK_LEFT_MODULE_STEER_OFFSET,
      BACK_RIGHT_MODULE_STEER_OFFSET
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public double getTrackwidth() {
    return TRACKWIDTH_METERS;
  }

  @Override
  public double getWheelbase() {
    return WHEELBASE_METERS;
  }

  @Override
  public double getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public double getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public Transform3d getRobotToCameraTransform() {
    return ROBOT_TO_CAMERA;
  }

  @Override
  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getAutoDriveKP() {
    return AUTO_DRIVE_P_CONTROLLER;
  }

  @Override
  public double getAutoDriveKI() {
    return AUTO_DRIVE_I_CONTROLLER;
  }

  @Override
  public double getAutoDriveKD() {
    return AUTO_DRIVE_D_CONTROLLER;
  }

  @Override
  public double getAutoTurnKP() {
    return AUTO_TURN_P_CONTROLLER;
  }

  @Override
  public double getAutoTurnKI() {
    return AUTO_TURN_I_CONTROLLER;
  }

  @Override
  public double getAutoTurnKD() {
    return AUTO_TURN_D_CONTROLLER;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public String getCameraName() {
    return CAMERA_NAME;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return PNEUMATICS_HUB_ID;
  }

  @Override
  public int getFlowSensorChannel() {
    return FLOW_SENSOR_CHANNEL;
  }

  @Override
  public int getRevHighPressureSensorChannel() {
    return REV_HIGH_PRESSURE_SENSOR_CHANNEL;
  }

  @Override
  public int getRevLowPressureSensorChannel() {
    return REV_LOW_PRESSURE_SENSOR_CHANNEL;
  }
}
