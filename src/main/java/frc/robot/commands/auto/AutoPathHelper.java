// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.HashMap;

/** Add your docs here. */
public class AutoPathHelper {

  // public static FollowPathWithEvents followPath(final String pathName, final HashMap<String,
  // Command> eventMap) {
  //     return new FollowPathWithEvents(getPathF, null, eventMap)
  // }

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
}
