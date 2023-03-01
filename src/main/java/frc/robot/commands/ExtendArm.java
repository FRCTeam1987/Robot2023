// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;

public class ExtendArm extends CommandBase {

  private final Arm ARM;
  private final int INCHES;

  public ExtendArm(final Arm ARM, final int INCHES) {
    this.ARM = ARM;
    this.INCHES = INCHES;
  }

  @Override
  public void initialize() {
    ARM.setArmLength(INCHES);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(ARM.getArmLength(), INCHES, 1));
  }
}
