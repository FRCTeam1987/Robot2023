// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Util;
import java.util.function.Supplier;

public class DriveToPose extends CommandBase {

  private final Drivetrain drive;
  private final Supplier<Pose2d> desiredPoseSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private SlewRateLimiter translationXSlewRate = new SlewRateLimiter(2.5);
  private SlewRateLimiter translationYSlewRate = new SlewRateLimiter(2.5);
  private SlewRateLimiter rotationSlewRate = new SlewRateLimiter(2);

  /** Creates a new DriveToPose. */
  public DriveToPose(final Drivetrain drive, final Supplier<Pose2d> desiredPoseSupplier) {
    this.drive = drive;
    this.desiredPoseSupplier = desiredPoseSupplier;
    xController = new PIDController(0.01, 0.0, 0.0);
    yController = new PIDController(0.01, 0.0, 0.0);
    thetaController = new PIDController(0.01, 0.0, 0.0);
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // check if long distance, if so, then exit
    if (drive.getPose().getX() > 2.5) {
      cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d currentPose = drive.getPose();
    final Pose2d desiredPose = desiredPoseSupplier.get();
    double xPercent = xController.calculate(currentPose.getX(), desiredPose.getX());
    double yPercent = yController.calculate(currentPose.getY(), desiredPose.getY());
    double thetaPercent =
        thetaController.calculate(
            currentPose.getRotation().getDegrees(), desiredPose.getRotation().getDegrees());
    xPercent = translationXSlewRate.calculate(-xPercent);
    yPercent = translationYSlewRate.calculate(-yPercent);
    thetaPercent = rotationSlewRate.calculate(-thetaPercent);

    drive.drive(
        xPercent * TeleopSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.5,
        yPercent * TeleopSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.5,
        thetaPercent * TeleopSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final Pose2d currentPose = drive.getPose();
    final Pose2d desiredPose = desiredPoseSupplier.get();
    return Util.isWithinTolerance(currentPose.getX(), desiredPose.getX(), 0.05)
        && Util.isWithinTolerance(currentPose.getY(), desiredPose.getY(), 0.05)
        && Util.isWithinTolerance(
            currentPose.getRotation().getDegrees(), desiredPose.getRotation().getDegrees(), 3);
  }
}
