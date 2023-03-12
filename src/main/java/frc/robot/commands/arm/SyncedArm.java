// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;

public class SyncedArm extends CommandBase {

  private final Arm arm;
  private DoubleSupplier desiredAngle;
  private DoubleSupplier desiredLength;

  private double initialAngle;
  private double initialLength;

  private double angleRange;
  private double lengthRange;

  /** Creates a new SyncedArm. */
  public SyncedArm(
      final Arm arm, final DoubleSupplier desiredAngle, final DoubleSupplier desiredLength) {
    this.arm = arm;
    this.desiredAngle = desiredAngle;
    this.desiredLength = desiredLength;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = arm.getArmAngle();
    initialLength = arm.getArmLength();
    angleRange = Math.abs(desiredAngle.getAsDouble() - initialAngle);
    lengthRange = Math.abs(desiredLength.getAsDouble() - initialLength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // final double currentAngle = arm.getArmAngle();
    // final double currentAnglePercent = Math.abs(currentAngle / angleRange);
    // final double currentLength = arm.getArmLength();
    // final double currentLengthPercent = Math.abs(currentLength / lengthRange);
    // final double desiredAngleValue = desiredAngle.getAsDouble();
    // arm.setArmAngle(desiredAngle.getAsDouble());
    // final double intermediateLength = isArmHome() ?
    //   desiredAngleValue - (Math.sqrt(1-currentAnglePercent) * desiredAngleValue)
    //   : desiredAngleValue - (Math.pow(1-currentAnglePercent, 2) * desiredAngleValue);
    // arm.setArmLength(findLength());

    if (Util.isWithinTolerance(arm.getArmAngle(), desiredAngle.getAsDouble(), 1)) {
      SmartDashboard.putNumber("Intermediate Length", desiredLength.getAsDouble());
    } else {
      SmartDashboard.putNumber(
          "Intermediate Length", MathUtil.clamp(findLength(), 0.0, desiredLength.getAsDouble()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DriverStation.reportWarning("SyncedArm interrupted!", false);
    }
    if (Util.isWithinTolerance(initialLength, initialAngle, angleRange)) {
      arm.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(arm.getArmAngle(), desiredAngle.getAsDouble(), 2)
        && Util.isWithinTolerance(arm.getArmLength(), desiredLength.getAsDouble(), 1);
  }

  private boolean isArmHome() {
    return Math.abs(arm.getArmLength()) < 5;
  }

  private double findLength() {
    final double currentAngle = arm.getArmAngle();
    final double currentAnglePercent = Math.abs((currentAngle - initialAngle) / angleRange);
    final double desiredAngleValue = desiredAngle.getAsDouble();
    arm.setArmAngle(desiredAngle.getAsDouble());

    SmartDashboard.putNumber("DesiredAngleValue", desiredAngleValue);
    SmartDashboard.putNumber("CurrentAnglePercent", currentAnglePercent);

    final double currentLength = arm.getArmLength();
    final boolean isRetract = currentLength > desiredLength.getAsDouble();
    return Math.abs(
        (desiredLength.getAsDouble() - currentLength)
            * (Math.abs(
                    isArmHome()
                        ? desiredAngleValue
                            - (Math.sqrt(1 - currentAnglePercent) * desiredAngleValue)
                        : desiredAngleValue
                            - (Math.pow(1 - currentAnglePercent, 2) * desiredAngleValue))
                / desiredAngleValue));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
