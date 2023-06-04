// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PositionConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreSequence extends SequentialCommandGroup {
  /** Creates a new ScoreSequence. */
  public AutoScoreSequence(
      final Arm arm,
      final Wrist wrist,
      final Claw claw,
      final Supplier<PositionConfig> positionConfigSupplier) {
    addCommands(
        new ScoreSequence(arm, wrist, positionConfigSupplier),
        new EjectGamePiece(claw).withTimeout(0.4),
        new GoHome(arm, wrist).withTimeout(2.0));
  }
}
