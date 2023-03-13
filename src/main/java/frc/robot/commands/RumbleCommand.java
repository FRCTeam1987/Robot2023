// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommand extends InstantCommand {
  /** Creates a new RumbleCommand. */
  public RumbleCommand(XboxController xboxController, double intensity, double duration) {
    xboxController.setRumble(RumbleType.kBothRumble, intensity);
    new WaitCommand(duration);
    xboxController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
