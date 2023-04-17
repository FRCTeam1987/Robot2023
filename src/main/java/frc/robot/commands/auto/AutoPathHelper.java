// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Util;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoPathHelper {

  private static final double DEFAULT_MAX_VELOCITY = 3;
  private static final double DEFAULT_MAX_ACCELERATION = 2.5;

  public static final Pose2d BABY_BIRD = new Pose2d(13.6, 7.25, Rotation2d.fromDegrees(90));
  public static final double GRID_X = 1.9;
  public static final List<Double> CONE_NODES_Y = List.of(0.51, 1.625, 2.19, 3.305, 3.865, 4.98);
  public static final double NODE_Y_TOLERANCE = 0.25;

  public static Command followPath(
      final Drivetrain drive, final String pathName, final HashMap<String, Command> eventMap) {
    return followPath(drive, pathName, eventMap, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION);
  }

  public static Command followPath(

      final Drivetrain drive,
      final String pathName,
      final HashMap<String, Command> eventMap,
      double maxVelocity,
      double maxAcceleration) {
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drive::getPose,
            drive::resetPose,
            drive.getKinematics(),
            new PIDConstants(
                AUTO_DRIVE_P_CONTROLLER, AUTO_DRIVE_I_CONTROLLER, AUTO_DRIVE_D_CONTROLLER),
            new PIDConstants(
                AUTO_TURN_P_CONTROLLER, AUTO_TURN_I_CONTROLLER, AUTO_TURN_D_CONTROLLER),
            drive::setSwerveModuleStates,
            eventMap,
            true,
            drive);
    return autoBuilder.fullAuto(
        PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVelocity, maxAcceleration)));
  }

  // Assumes creating a new command upon button press and throw away...
  // Might want to consider using suppliers for poses
  // Need a way to cancel this command if it goes awry mid-match
  public static Command onTheFlyPath(
      final Drivetrain drive,
      Supplier<Pose2d> initialPoseSupplier,
      Supplier<Pose2d> finalPoseSupplier) {
    return new DynamicPPSwerveControllerCommand(
        () -> {
          final double xDifference =
              finalPoseSupplier.get().getX() - initialPoseSupplier.get().getX();
          final double yDifference =
              finalPoseSupplier.get().getY() - initialPoseSupplier.get().getY();
          Rotation2d initialHeading = new Rotation2d();
          Rotation2d finalHeading = new Rotation2d();
          if (Math.abs(xDifference) > Math.abs(yDifference)) {
            if (xDifference > 0) {
              initialHeading = Rotation2d.fromDegrees(180);
              finalHeading = Rotation2d.fromDegrees(0);
            } else {
              initialHeading = Rotation2d.fromDegrees(0);
              finalHeading = Rotation2d.fromDegrees(180);
            }
          } else {
            if (yDifference > 0) {
              initialHeading = Rotation2d.fromDegrees(-90);
              finalHeading = Rotation2d.fromDegrees(90);
            } else {
              initialHeading = Rotation2d.fromDegrees(90);
              finalHeading = Rotation2d.fromDegrees(-90);
            }
          }
          return PathPlanner.generatePath(
              new PathConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION),
              new PathPoint(
                  initialPoseSupplier.get().getTranslation(),
                  initialHeading,
                  initialPoseSupplier
                      .get()
                      .getRotation()), // position, heading(direction of travel), holonomic rotation
              new PathPoint(
                  finalPoseSupplier.get().getTranslation(),
                  finalHeading,
                  finalPoseSupplier
                      .get()
                      .getRotation()) // position, heading(direction of travel), holonomic rotation
              );
        },
        drive::getPose, // Pose supplier
        drive.getKinematics(), // SwerveDriveKinematics
        new PIDController(
            AUTO_DRIVE_P_CONTROLLER,
            AUTO_DRIVE_I_CONTROLLER,
            AUTO_DRIVE_D_CONTROLLER), // X controller. Tune these values for your robot. Leaving
        // them 0 will only use feedforwards.
        new PIDController(
            AUTO_DRIVE_P_CONTROLLER,
            AUTO_DRIVE_I_CONTROLLER,
            AUTO_DRIVE_D_CONTROLLER), // Y controller (usually the same values as X controller)
        new PIDController(
            AUTO_TURN_P_CONTROLLER,
            AUTO_TURN_I_CONTROLLER,
            AUTO_TURN_D_CONTROLLER), // Rotation controller. Tune these values for your robot.
        // Leaving them 0 will only use feedforwards.
        drive::setSwerveModuleStates, // Module states consumer
        true, // Should the path be automatically mirrored depending on alliance color. Optional,
        // defaults to true
        drive // Requires this drive subsystem
        );
  }

  public static Command driveToBabyBird(final Drivetrain drive) {
    return onTheFlyPath(drive, drive::getPose, () -> BABY_BIRD);
  }

  public static Command driveToClosestConeNode(final Drivetrain drive) {
    return onTheFlyPath(
        drive,
        drive::getPose,
        () -> {
          final Pose2d currentPose = drive.getPose();
          if (currentPose.getX() > 2.5) {
            DriverStation.reportWarning("X too far away, not auto driving to cone node", false);
            return drive.getPose();
          }
          final double currentY = currentPose.getY();
          final double closestNodeY =
              CONE_NODES_Y.stream()
                  .filter((Double y) -> Util.isWithinTolerance(currentY, y, NODE_Y_TOLERANCE))
                  .findFirst()
                  .get();
          return new Pose2d(GRID_X, closestNodeY, Rotation2d.fromDegrees(180));
        });
  }
}
