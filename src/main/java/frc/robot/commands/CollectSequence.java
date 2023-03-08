// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CollectConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectSequence extends SequentialCommandGroup {

  /** Creates a new CollectSequence. */
  public CollectSequence(
      final Arm arm,
      final Wrist wrist,
      final Claw claw,
      final Supplier<CollectConfig> collectConfig) {
    addCommands(
        new ParallelCommandGroup(
            new RotateArmSupplier(arm, () -> collectConfig.get().armRotation),
            new SetWristPositionSupplier(wrist, () -> collectConfig.get().wristRotation)),
        new ExtendArmSupplier(arm, () -> collectConfig.get().armLength),
        new ConditionalCommand(
            new CollectGamePiece(claw, GamePiece.CUBE),
            new CollectGamePiece(claw, GamePiece.CONE),
            () -> collectConfig.get().gamePiece == GamePiece.CUBE),
        new ExtendArm(arm, Arm.HOME_EXTENSION),
        new ParallelCommandGroup(
            new RotateArm(arm, Arm.HOME_ROTATION),
            new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
        new InstantCommand(() -> arm.setExtensionNominal(), arm));
  }
}