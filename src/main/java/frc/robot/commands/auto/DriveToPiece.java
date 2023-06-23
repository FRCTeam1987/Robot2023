// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveToPiece extends CommandBase {
  /** Creates a new DriveToPiece. */
  private DoubleSupplier velocitySupplier;

  private final Drivetrain drivetrain;

  private String limelight;
  private Pose2d initialPose;
  private static final double kP = 0.03; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.01; // PID derivative gain
  private static final double kToleranceDegrees = 2.0; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 1; // In Meters
  boolean isDistanceTraveledToFar = distanceTraveled() > maximumAllowableDistance;

  private final PIDController rotationController;

  public DriveToPiece(
      final Drivetrain drivetrain, final DoubleSupplier velocitySupplier, String limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocitySupplier = velocitySupplier;
    this.drivetrain = drivetrain;
    this.limelight = limelight;

    // Create the PID controller
    rotationController = new PIDController(kP, kI, kD);
    rotationController.setTolerance(kToleranceDegrees);

    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Apply the output to the swerve
    this.initialPose = drivetrain.getPose();
    rotationController.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isDistanceTraveledToFar = distanceTraveled() > maximumAllowableDistance;
    if (LimelightHelpers.getTV(limelight)) {
      drivetrain.disableFieldRelative();
      double rotationalVelocity =
          rotationController.calculate(
              drivetrain.getRotation().getDegrees(),
              drivetrain.getRotation().getDegrees() - LimelightHelpers.getTX(limelight));
      final double velocity = velocitySupplier.getAsDouble();
      drivetrain.drive(velocity, velocity, rotationalVelocity, true);
    } else {
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.enableFieldRelative();
    if (isDistanceTraveledToFar && !interrupted) {
      drivetrain
          .stop(); // Stop the drive train only when command isn't interrupted and robot drove too
      // far
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledToFar; // Saftey if command is improperly called or fails to collect
    // gamepiece.
  }

  private double distanceTraveled() {
    Translation2d currentPose = drivetrain.getPose().getTranslation();
    return currentPose.getDistance(initialPose.getTranslation());
  }
}
