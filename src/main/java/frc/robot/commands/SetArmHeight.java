// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PositionConfigs;
import frc.robot.RobotContainer.Height;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmHeight extends InstantCommand {

  Wrist wrist;
  Claw claw;
  Height height;
  Arm arm;
  XboxController controller;

  public SetArmHeight(
      Height height, Arm arm, Wrist wrist, Claw claw, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.arm = arm;
    this.claw = claw;
    this.height = height;
    this.controller = controller;
    addRequirements(this.wrist, this.arm, this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(height.toString());
    if (height == Height.FLOOR) {
      new ConditionalCommand(
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.BACK_CONE_FLOOR),
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.BACK_CUBE_FLOOR),
          () -> claw.isCone());
    } else if (height == Height.MEDIUM) {
      new ConditionalCommand(
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_MEDIUM),
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_MEDIUM),
          () -> claw.isCone());
    } else if (height == Height.HIGH) { // Defaults to high if not set
      new ConditionalCommand(
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CONE_TOP),
          new ScoreSequence(arm, wrist, claw, () -> PositionConfigs.FRONT_CUBE_TOP),
          () -> claw.isCone());
    } else {
      new RumbleCommand(controller, 1, 0.5);
    }
  }
}
