// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw.GamePiece;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean ADVANTAGE_KIT_ENABLED = false;
  public static final boolean TUNING_MODE = false;

  // FIXME: If Limelight is used, specify the pipeline for detecting AprilTags
  public static final int LIMELIGHT_PIPELINE = 1;
  public static final int WRIST_OFFSET = 400; // -217 // 606
  public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  public static final double AUTO_TURN_P_CONTROLLER = 10.0;
  public static final double AUTO_TURN_I_CONTROLLER = 0.0;
  public static final double AUTO_TURN_D_CONTROLLER = 0.0;
  public static final ShuffleboardTab TAB_VISION = Shuffleboard.getTab("Vision");
  public static final ShuffleboardTab TAB_MAIN = Shuffleboard.getTab("Main");
  public static final ShuffleboardTab TAB_ARM = Shuffleboard.getTab("Arm");
  public static final ShuffleboardTab TAB_WRIST = Shuffleboard.getTab("Wrist");
  public static final ShuffleboardTab TAB_CLAW = Shuffleboard.getTab("Claw");
  public static final ShuffleboardTab TAB_COMMANDS = Shuffleboard.getTab("Commands");
  public static final ShuffleboardTab TAB_MATCH = Shuffleboard.getTab("Match");
  private static final RobotType ROBOT =
      RobotBase.getRuntimeType().equals(RuntimeType.kRoboRIO)
          ? RobotType.ROBOT_2023_TEST
          : RobotType.ROBOT_2023_COMP;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_DEFAULT;
      } else {
        return ROBOT;
      }
    } else {
      if (ROBOT == RobotType.ROBOT_SIMBOT && !ADVANTAGE_KIT_ENABLED) {
        invalidRobotAlert.set(true);
      } else {
        return ROBOT;
      }
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023_TEST:
      case ROBOT_2023_COMP:
      case ROBOT_DEFAULT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_2023_TEST,
    ROBOT_2023_COMP,
    ROBOT_DEFAULT,
    ROBOT_SIMBOT
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

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
    public static final PositionConfig BACK_CUBE_FLOOR_LONG =
        new PositionConfig(26, 93.8, 1713 + WRIST_OFFSET, GamePiece.CUBE);

    public static final PositionConfig BACK_CUBE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, 101.7, 1550 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig BACK_CONE_FLOOR =
        new PositionConfig(Arm.HOME_EXTENSION, 90.9, 1805 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CONE_FLOOR_TIPPED =
        new PositionConfig(Arm.HOME_EXTENSION, 108, 1347 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_MEDIUM =
        new PositionConfig(17, -43, 489 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_CONE_TOP =
        new PositionConfig(37, -49, 706 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig AUTO_FRONT_CONE_TOP =
        new PositionConfig(39, -47, 321 + WRIST_OFFSET, GamePiece.CONE); // -49 rotation, 656 wrist
    public static final PositionConfig FRONT_CUBE_MEDIUM =
        new PositionConfig(1, -47, 1016 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig FRONT_CUBE_TOP =
        new PositionConfig(20, -48.5, 1077 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig AUTO_FRONT_CUBE_TOP =
        new PositionConfig(
            18, -46.5, 1177 + WRIST_OFFSET, GamePiece.CUBE); // wrist 1027, arm angle -50
    public static final PositionConfig BACK_CONE_TOP =
        new PositionConfig(
            38, 49.5, 2800 + WRIST_OFFSET, GamePiece.CONE); // length 35, rotation 49.5, wrist
    public static final PositionConfig BACK_CONE_MEDIUM =
        new PositionConfig(22, 47.9, 701 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_CUBE_TOP =
        new PositionConfig(21, 51, 2360 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig BACK_CUBE_MEDIUM =
        new PositionConfig(0, 51.5, 2383 + WRIST_OFFSET, GamePiece.CUBE);
    public static final PositionConfig FRONT_SINGLE_SUBSTATION =
        new PositionConfig(0, -72, 2330 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig FRONT_DOUBLE_SUBSTATION =
        new PositionConfig(17, -33, 1270 + WRIST_OFFSET, GamePiece.CONE);

    public static final PositionConfig BACK_SINGLE_SUBSTATION =
        new PositionConfig(0, 81, 750 + WRIST_OFFSET, GamePiece.CONE);
    public static final PositionConfig BACK_DOUBLE_SUBSTATION =
        new PositionConfig(18, 27, 2570 + WRIST_OFFSET, GamePiece.CONE);
  }
}
