// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class ParallelCommandTest extends ParallelCommandGroup {

  public ParallelCommandTest(
      final Arm ARM,
      final Wrist WRIST,
      final int INCHES,
      final double ANGLE,
      final boolean ROTATION) {
    addCommands(
        new ExtendArm(ARM, INCHES), new RotateArm(ARM, ANGLE), new FlipWrist(WRIST, ROTATION));
  }
}
