// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome(final Arm arm, final Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        new ConditionalCommand(
            new ExtendArmSupplier(arm, () -> 2),
            new InstantCommand(),
            () -> arm.getArmLength() > 4),
        new ParallelCommandGroup(
            new RotateArm(arm, Arm.HOME_ROTATION),
            new SetWristPosition(Wrist.ANGLE_STRAIGHT, wrist)),
        new InstantCommand(arm::setExtensionNominal, arm));
  }
}
