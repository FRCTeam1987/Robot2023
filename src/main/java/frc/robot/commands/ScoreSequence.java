// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PositionConfig;
import frc.robot.commands.arm.SetArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreSequence extends SequentialCommandGroup {
  /** Creates a new ScoreSequence. */
  public ScoreSequence(
    final Arm arm,
      final Wrist wrist,
      final Claw claw,
      final Supplier<PositionConfig> PositionConfig) {
    addCommands(
            new SetArm(
                arm, () -> PositionConfig.get().armRotation, () -> PositionConfig.get().armLength),
            new SetWristPositionSupplier(wrist, () -> PositionConfig.get().wristRotation),
        new ReleaseGamePiece(claw),
        new ParallelCommandGroup(
            new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION),
            new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
        new InstantCommand(() -> arm.setExtensionNominal(), arm));
  }
}

