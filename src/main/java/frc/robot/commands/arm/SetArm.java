// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SetArm extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier angleSupplier;
  private final DoubleSupplier lengthSupplier;
  private final BooleanSupplier isReturning;

  /** Creates a new SetArm. */
  public SetArm(
      final Arm arm,
      final DoubleSupplier angleSupplier,
      final DoubleSupplier lengthSupplier,
      final BooleanSupplier isReturning) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;
    this.lengthSupplier = lengthSupplier;
    this.isReturning = isReturning;
    addRequirements(this.arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO set arm angle only when more retracted, this assumes it is retracted and it might not be

    final double desiredLength = lengthSupplier.getAsDouble();
    final double desiredAngle = angleSupplier.getAsDouble();
    final boolean returning = isReturning.getAsBoolean();

    if (returning) {
      arm.setArmLength(desiredLength);
      if (isLengthOnTarget(15)) {
        arm.setArmAngle(angleSupplier.getAsDouble());
      }
    } else {
      arm.setArmAngle(desiredAngle);
      if (isAngleOnTarget(35) && !isLengthOnTarget(1)) {
        arm.setArmLength(desiredLength);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.stop();
    }
    if (Util.isWithinTolerance(arm.getArmAngle(), 0, 2)
        && Util.isWithinTolerance(arm.getArmLength(), 1, 2)) {
      arm.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAngleOnTarget(3.5) && isLengthOnTarget(2);
  }

  private boolean isAngleOnTarget(final double tolerance) {
    return Util.isWithinTolerance(arm.getArmAngle(), angleSupplier.getAsDouble(), tolerance);
  }

  private boolean isLengthOnTarget(final double tolerance) {
    return Util.isWithinTolerance(arm.getArmLength(), lengthSupplier.getAsDouble(), tolerance);
  }
}
