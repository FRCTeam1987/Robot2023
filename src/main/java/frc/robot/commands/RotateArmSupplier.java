// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;

public class RotateArmSupplier extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier angleSupplier;

  public RotateArmSupplier(final Arm arm, final DoubleSupplier angle) {
    this.arm = arm;
    this.angleSupplier = angle;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    arm.setArmAngle(angleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("Not interrupted, holding current angle!");
      arm.holdCurrentAngle(angleSupplier.getAsDouble());
    }
  }

  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(arm.getArmAngle(), angleSupplier.getAsDouble(), 1);
  }
}
