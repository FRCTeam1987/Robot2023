// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw.GamePiece;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double FALCON_MAX_RPM = 6380.0;
  public static final int CLAW_MOTOR_ID = 8;

  public static final int WRIST_OFFSET = 606;
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
  public static final int GYRO_ID = 0;

  // FIXME: update robot dimensions
  public static final double TRACKWIDTH_METERS = Units.inchesToMeters(18.75); // 22.5 inches
  public static final double WHEELBASE_METERS = Units.inchesToMeters(21.75); // 23.5 inches
  public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

  // FIXME: tune PID values for the angle and drive motors for the swerve modules

  /* Angle Motor PID Values */
  public static final double ANGLE_KP = 0.6;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 12.0;
  public static final double ANGLE_KF = 0.0;

  /* Drive Motor PID Values */
  public static final double DRIVE_KP = 0.1;
  public static final double DRIVE_KI = 0.0;
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KF = 0.0;

  // FIXME: characterize the drivetrain and update these constants
  public static final double DRIVE_KS = 0.55493;
  public static final double DRIVE_KV = 2.3014;
  public static final double DRIVE_KA = 0.12872;

  // FIXME: determine maximum velocities empirically
  public static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.25;
  // FIXME: specify the name of the CANivore CAN FD bus as appropriate (an empty string uses the
  // default CAN bus)
  public static final String CAN_BUS_NAME = "canfd";

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));

  public static final int MAX_VOLTAGE = 12;
  // FIXME: specify maximum velocity and acceleration and tune PID values for auto paths

  public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  public static final double AUTO_TURN_P_CONTROLLER = 25.0;
  public static final double AUTO_TURN_I_CONTROLLER = 0.0;
  public static final double AUTO_TURN_D_CONTROLLER = 0.0;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS =
      Units.inchesToMeters(18.75); // 22.5 inches
  public static final double DRIVETRAIN_WHEELBASE_METERS =
      Units.inchesToMeters(21.75); // 23.5 inches
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED =
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 3.0;

  public static final boolean ADVANTAGE_KIT_ENABLED = false; // FOREVER MOREEEEEEEE
  // leave this constant so i can have a legacy -Christopher
  public static final boolean TUNING_MODE = false;

  // FIXME: If Limelight is used, specify the pipeline for detecting AprilTags
  public static final int LIMELIGHT_PIPELINE = 1;

  public static final ShuffleboardTab TAB_VISION = Shuffleboard.getTab("Vision");
  public static final ShuffleboardTab TAB_MAIN = Shuffleboard.getTab("Main");
  public static final ShuffleboardTab TAB_DRIVETRAIN = Shuffleboard.getTab("Drivetrain");
  public static final ShuffleboardTab TAB_ARM = Shuffleboard.getTab("Arm");
  public static final ShuffleboardTab TAB_WRIST = Shuffleboard.getTab("Wrist");
  public static final ShuffleboardTab TAB_CLAW = Shuffleboard.getTab("Claw");
  public static final ShuffleboardTab TAB_COMMANDS = Shuffleboard.getTab("Commands");

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static class PositionConfig {
    public final int armLength;
    public final double armRotation;
    public final int wristRotation;
    public final GamePiece gamePiece;

    public PositionConfig(
        final int length,
        final double rotation,
        final int wristRotation,
        final GamePiece gamePiece) {
      this.armLength = length;
      this.armRotation = rotation;
      this.wristRotation = wristRotation;
      this.gamePiece = gamePiece;
    }
  }

  public static class PositionConfigs {
    public static final PositionConfig TEST_POS =
        new PositionConfig(Arm.HOME_EXTENSION, 45, 1692 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig TEST_NEG =
        new PositionConfig(Arm.HOME_EXTENSION, -45, 1692 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CUBE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, -105.4, 1780 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig FRONT_CONE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, -102.4, 2072 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_FLOOR_TIPPED =
        new PositionConfig(Arm.HOME_EXTENSION, -107.3, 1906 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_FLOOR_TIPPED_LONG =
        new PositionConfig(23, -102.4, 2130 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CUBE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, 101.7, 1550 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig BACK_CONE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, 90.9, 1755 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CONE_FLOOR_TIPPED =
        new PositionConfig(Arm.HOME_EXTENSION, 108, 1347 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_MEDIUM =
        new PositionConfig(17, -43, 489 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_TOP =
        new PositionConfig(35, -43, 575 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CUBE_MEDIUM =
        new PositionConfig(1, -47, 1016 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig FRONT_CUBE_TOP =
        new PositionConfig(
            20, -48.5, 1077 + WRIST_OFFSET, GamePiece.CUBE); // wrist 1027, arm angle -50
    public static final PositionConfig BACK_CONE_TOP =
        new PositionConfig(35, 49.5, 2800 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CONE_MEDIUM =
        new PositionConfig(22, 47.9, 3260 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CUBE_TOP =
        new PositionConfig(21, 51, 2360 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig BACK_CUBE_MEDIUM =
        new PositionConfig(0, 51.5, 2383 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig FRONT_SINGLE_SUBSTATION =
        new PositionConfig(0, -72, 2330 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_DOUBLE_SUBSTATION =
        new PositionConfig(17, -33, 1270 + WRIST_OFFSET, GamePiece.CONE);

    public static final PositionConfig BACK_SINGLE_SUBSTATION =
        new PositionConfig(0, 51, 2093, GamePiece.CONE);
    public static final PositionConfig BACK_DOUBLE_SUBSTATION =
        new PositionConfig(18, 27, 2570 + WRIST_OFFSET, GamePiece.CONE);
  }
}
