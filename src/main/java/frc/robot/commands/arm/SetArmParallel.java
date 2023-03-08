// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arm.ArmConstants.CLAW_HEIGHT_INCHES;
import static frc.robot.subsystems.arm.ArmConstants.MAX_VERTICAL_EXTENSION_INCHES;

public class SetArmParallel extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier angleSupplier;
  private final DoubleSupplier lengthSupplier;

  /** Creates a new SetArm. */
  public SetArmParallel(
      final Arm arm, final DoubleSupplier angleSupplier, final DoubleSupplier lengthSupplier) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;
    this.lengthSupplier = lengthSupplier;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // arm.setArmAngle(angleSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmAngle(angleSupplier.getAsDouble());
    if (Math.sin(angleSupplier.getAsDouble()) * (lengthSupplier.getAsDouble() + CLAW_HEIGHT_INCHES) > MAX_VERTICAL_EXTENSION_INCHES) {
      arm.setArmLength(arm.getArmLength());
    } else {
      arm.setArmLength(lengthSupplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.stop();
      return;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAngleOnTarget(4) && isLengthOnTarget(2);
  }

  private boolean isAngleOnTarget(final double tolerance) {
    return Util.isWithinTolerance(arm.getArmAngle(), angleSupplier.getAsDouble(), tolerance);
  }

  private boolean isLengthOnTarget(final double tolerance) {
    return Util.isWithinTolerance(arm.getArmLength(), lengthSupplier.getAsDouble(), tolerance);
  }
}
