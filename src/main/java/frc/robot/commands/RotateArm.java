// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;

public class RotateArm extends CommandBase {

  private final Arm ARM;
  private final double ANGLE;

  public RotateArm(final Arm ARM, final double ANGLE) {
    this.ARM = ARM;
    this.ANGLE = ANGLE;
    addRequirements(this.ARM);
  }

  @Override
  public void initialize() {
    ARM.setArmAngle(ANGLE);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("Not interrupted, holding current angle!");
      ARM.holdCurrentAngle(ANGLE);
    }
  }

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(ARM.getArmAngle(), ANGLE, 0.5));
  }
}
