// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmProfile;
import frc.robot.subsystems.claw.Claw.GamePiece;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final ArmProfile ARM_A = new ArmProfile(1012, -322.037); // +1
  public static final ArmProfile ARM_B = new ArmProfile(687, -201); // +1.412
  public static ArmProfile INSTALLED_ARM = true ? ARM_A : ARM_B;

  public static final boolean ADVANTAGE_KIT_ENABLED = true;
  public static final boolean TUNING_MODE = false;

  // If Limelight is used, specify the pipeline for detecting AprilTags
  public static final int LIMELIGHT_PIPELINE = 0;
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
  public static final ShuffleboardTab TAB_TEST = Shuffleboard.getTab("Test");
  public static final double MAX_ANGULAR_VELOCITY = 10.0;

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static class PositionConfig {
    public final double armLength;
    public final double armRotation;
    public final int wristRotation;
    public final GamePiece gamePiece;

    public PositionConfig(
        final double length,
        final double rotation,
        final int wristRotation,
        final GamePiece gamePiece) {
      this.armLength = length;
      this.armRotation = rotation;
      this.wristRotation = wristRotation;
      this.gamePiece = gamePiece;
    }

    public double getWristRotation() {
      return this.wristRotation;
    }

    public double getArmRotation() {
      return this.armRotation;
    }

    public double getArmLength() {
      return this.armLength;
    }
  }

  public static class PositionConfigs {
    public static final PositionConfig AUTO_ALMOST_FLOOR_CUBE =
        new PositionConfig(
            Arm.HOME_EXTENSION, -80, 1550 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig AUTO_FRONT_CUBE_TOP =
        new PositionConfig(18, -57.8, 2045 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig
        AUTO_BACK_CONE_FLOOR = // TODO might not be necessary, maybe use regular position
        new PositionConfig(1, 83, 1850 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig BACK_CONE_FLOOR =
        new PositionConfig(1, 82.7, 1735 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig
        BACK_CONE_FLOOR_TIPPED = // not a good angle with new LL mounts
        new PositionConfig(10.25, 105.25, 1460 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig BACK_CONE_MEDIUM =
        new PositionConfig(22, 47.9, 701 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig BACK_CONE_TOP =
        new PositionConfig(38, 49.5, 2800 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig BACK_CUBE_FLOOR =
        new PositionConfig(4, 98.2, 2731 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig BACK_CUBE_FLOOR_LONG =
        new PositionConfig(27, 88, 2076 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig BACK_CUBE_MEDIUM =
        new PositionConfig(0, 51.5, 2383 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig BACK_CUBE_TOP =
        new PositionConfig(21, 51, 2360 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig BACK_SINGLE_SUBSTATION =
        new PositionConfig(0, 81, 750 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_FLOOR =
        new PositionConfig(
            Arm.HOME_EXTENSION, -102.4, 2072 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_FLOOR_TIPPED =
        new PositionConfig(
            Arm.HOME_EXTENSION, -107.3, 1896 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_FLOOR_TIPPED_LONG =
        new PositionConfig(23, -102.4, 2130 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_MEDIUM =
        new PositionConfig(20.7, -43.5, 300 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_MEDIUM_AUTO =
        new PositionConfig(18, -40, 125 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_TOP =
        new PositionConfig(37.5, -45.6, 433 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CONE_TOP_AUTO =
        new PositionConfig(37.5, -43.6, 340 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_CUBE_FLOOR =
        new PositionConfig(
            Arm.HOME_EXTENSION, -87, 1738 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig FRONT_CUBE_MEDIUM =
        new PositionConfig(3, -65.4, 2128 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig FRONT_CUBE_MEDIUM_AUTO =
        new PositionConfig(6, -65.4, 2128 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig FRONT_CUBE_MEDIUM_EXTENDED =
        new PositionConfig(5, -47, 1016 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig FRONT_CUBE_TOP =
        new PositionConfig(22, -57.8, 2045 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig FRONT_CUBE_TOP_AUTO =
        new PositionConfig(24, -57.8, 2045 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig BACK_DOUBLE_SUBSTATION =
        new PositionConfig(22.25, 27, 2390 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig FRONT_SINGLE_SUBSTATION =
        new PositionConfig(0, -72, 2330 + INSTALLED_ARM.getWristOffset(), GamePiece.CONE);

    public static final PositionConfig SPIT_BACK_CUBE_FLOOR_LONG =
        new PositionConfig(27.2, -90.8, 1454 + INSTALLED_ARM.getWristOffset(), GamePiece.CUBE);

    public static final PositionConfig LAUNCH_LAST_AUTO_CUBE =
        new PositionConfig(6, -28.0, 1970, GamePiece.CUBE);
  }

  public static class OnTheFly {
    public static final Pose2d BABY_BIRD = new Pose2d(13.6, 7.25, Rotation2d.fromDegrees(90));
    public static final double GRID_X = 1.9;
    public static final List<Double> CONE_NODES_Y = List.of(0.51, 1.625, 2.19, 3.305, 3.865, 4.98);
    public static final List<Pose2d> CONE_NODES_POSE =
        List.of(
            new Pose2d(GRID_X, 0.51, new Rotation2d()),
            new Pose2d(GRID_X, 1.625, new Rotation2d()));
    public static final double NODE_Y_TOLERANCE = 0.25;
  }
}
