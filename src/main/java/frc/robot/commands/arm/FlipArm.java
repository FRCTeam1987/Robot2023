// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;

public class FlipArm extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier angleSupplier;
  /** Creates a new FlipArm. */
  public FlipArm(final Arm arm, final DoubleSupplier angleSupplier) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO set arm angle only when more retracted, this assumes it is retracted and it might not be
    if (-arm.getArmAngle() < -90 || -arm.getArmAngle() > 90)
      arm.setArmAngle(arm.getArmAngle());
    else {
      arm.setArmAngle(-arm.getArmAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAngleOnTarget(3);
  }

  private boolean isAngleOnTarget(final double tolerance) {
    return Util.isWithinTolerance(arm.getArmAngle(), angleSupplier.getAsDouble(), tolerance);
  }

}
