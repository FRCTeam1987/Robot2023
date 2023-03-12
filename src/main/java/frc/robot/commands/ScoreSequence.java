// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PositionConfig;
import frc.robot.commands.arm.SetArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreSequence extends ParallelCommandGroup {
  /** Creates a new ScoreSequence. */
  public ScoreSequence(
      final Arm arm,
      final Wrist wrist,
      final Claw claw,
      final Supplier<PositionConfig> positionConfigSupplier) {
    addCommands(
        new SetArm(
            arm,
            () -> positionConfigSupplier.get().armRotation,
            () -> positionConfigSupplier.get().armLength,
            () -> false),
        new SetWristPositionSupplier(wrist, () -> positionConfigSupplier.get().wristRotation)

        // new EjectGamePiece(claw).withTimeout(0.5),
        // new ParallelCommandGroup(
        //     new SetArm(arm, () -> Arm.HOME_ROTATION, () -> Arm.HOME_EXTENSION, () -> true),
        //     new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
        // new InstantCommand(() -> arm.setExtensionNominal(), arm)
        );
  }
}
