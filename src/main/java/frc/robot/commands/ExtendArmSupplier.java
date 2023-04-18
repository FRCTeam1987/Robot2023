// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;
import java.util.function.IntSupplier;

public class ExtendArmSupplier extends CommandBase {

  private final Arm arm;
  private final IntSupplier inches;

  public ExtendArmSupplier(final Arm arm, final IntSupplier inches) {
    this.arm = arm;
    this.inches = inches;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    arm.setArmLength(inches.getAsInt());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(arm.getArmLength(), inches.getAsInt(), 2);
  }
}
