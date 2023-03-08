// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
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

  public static final boolean TUNING_MODE = false;

  // FIXME: If Limelight is used, specify the pipeline for detecting AprilTags
  public static final int LIMELIGHT_PIPELINE = 1;

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

  public static class CollectConfig {
    public final int armLength;
    public final double armRotation;
    public final int wristRotation;
    public final GamePiece gamePiece;

    public CollectConfig(
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

  public static class CollectConfigs {
    public static final CollectConfig TEST_POS =
        new CollectConfig(Arm.HOME_EXTENSION, 45, 1692, GamePiece.CUBE);
    public static final CollectConfig TEST_NEG =
        new CollectConfig(Arm.HOME_EXTENSION, -45, 1692, GamePiece.CONE);
    public static final CollectConfig FRONT_CUBE_FLOOR =
        new CollectConfig(Arm.HOME_EXTENSION, -105.4, 1692, GamePiece.CUBE);
    public static final CollectConfig FRONT_CONE_FLOOR =
        new CollectConfig(Arm.HOME_EXTENSION, -102.4, 1853, GamePiece.CONE);
    public static final CollectConfig FRONT_CONE_FLOOR_TIPPED =
        new CollectConfig(Arm.HOME_EXTENSION, -107.3, 1806, GamePiece.CONE);
    public static final CollectConfig FRONT_CONE_FLOOR_TIPPED_LONG =
        new CollectConfig(22, -101.5, 1822, GamePiece.CONE);
    public static final CollectConfig BACK_CUBE_FLOOR =
        new CollectConfig(Arm.HOME_EXTENSION, 101.7, 2329, GamePiece.CUBE);
    public static final CollectConfig BACK_CONE_FLOOR =
        new CollectConfig(Arm.HOME_EXTENSION, 90.9, 2494, GamePiece.CONE);
    public static final CollectConfig BACK_CONE_FLOOR_TIPPED =
        new CollectConfig(Arm.HOME_EXTENSION, 105.5, 2068, GamePiece.CONE);
  }
}
