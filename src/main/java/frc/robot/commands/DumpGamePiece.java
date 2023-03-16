// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.wrist.Wrist;

public class DumpGamePiece extends SequentialCommandGroup {

  public DumpGamePiece(final Wrist wrist, final Claw claw) {
    addCommands(
        new SetWristPosition(3289, wrist),
        new EjectGamePiece(claw).withTimeout(.66),
        new SetWristPosition(2289, wrist));
  }
}
