// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.Height;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmHeight extends InstantCommand {

  Wrist wrist;
  Claw claw;
  Height height;
  Arm arm;
  CommandXboxController controller;

  public SetArmHeight(
      Height height, Arm arm, Wrist wrist, Claw claw, CommandXboxController controller) {
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
    DriverStation.reportWarning("arm height:" + height.toString(), false);
  }
}
