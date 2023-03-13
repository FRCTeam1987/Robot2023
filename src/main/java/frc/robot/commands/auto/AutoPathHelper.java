// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.HashMap;

/** Add your docs here. */
public class AutoPathHelper {

  // public static FollowPathWithEvents followPath(final String pathName, final HashMap<String,
  // Command> eventMap) {
  //     return new FollowPathWithEvents(getPathF, null, eventMap)
  // }

  public static Command followPath(
      final DrivetrainSubsystem drive,
      final String pathName,
      final HashMap<String, Command> eventMap) {
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drive::getPose,
            drive::setPose,
            new PIDConstants(5.0, 0, 0),
            new PIDConstants(1.0, 0, 0),
            drive::setChassisSpeeds,
            eventMap,
            true,
            drive);
    return autoBuilder.fullAuto(PathPlanner.loadPathGroup(pathName, new PathConstraints(2, 2)));
  }
}
