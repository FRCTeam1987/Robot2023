// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.claw.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectGamePiece extends SequentialCommandGroup {
  private static final double collectPercent = 0.5;
  private static final double collectedCurrent = 10;

  public CollectGamePiece(Claw claw) {
    addCommands(
        new StartEndCommand(
                () -> {
                  claw.setRollerSpeed(collectPercent);
                  System.out.print("start");
                },
                () -> {
                  claw.stopRollers();
                  System.out.print("End");
                },
                claw)
            .until(() -> claw.getCurrent() > collectedCurrent));
  }
}
