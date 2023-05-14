// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;

public class RotateArm extends CommandBase {

  private final Arm arm;
  private final double angle;

  public RotateArm(final Arm arm, final double angle) {
    this.arm = arm;
    this.angle = angle;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    arm.setArmAngle(angle);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      DriverStation.reportWarning("Not interrupted, holding current arm angle!", false);
      arm.holdCurrentAngle(angle);
    }
  }

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(arm.getArmAngle(), angle, 0.5));
  }
}
