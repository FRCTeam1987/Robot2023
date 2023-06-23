// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveToPiece extends CommandBase {
  /** Creates a new DriveToPiece. */
  private DoubleSupplier velocitySupplier;

  private final Drivetrain drivetrain;

  private String limelight = "limelight-fr";
  private double initialXPose;
  private double initialYPose;
  private static final double kP = 0.03; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.01; // PID derivative gain
  private static final double kToleranceDegrees = 2.0; // Tolerance for reaching the desired angle

  private final PIDController pidController;

  public DriveToPiece(
      final Drivetrain drivetrain, final DoubleSupplier velocitySupplier, String limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocitySupplier = velocitySupplier;
    this.drivetrain = drivetrain;

    // Create the PID controller
    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(kToleranceDegrees);

    // Add the required subsystem (SwerveDriveSubsystem) to the command's requirements
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Apply the output to the swerve
    this.initialXPose = drivetrain.getPoseX();
    this.initialYPose = drivetrain.getPoseY();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight)) {
      double output =
          pidController.calculate(
              drivetrain.getRotation().getDegrees(),
              drivetrain.getRotation().getDegrees() - LimelightHelpers.getTX(limelight));

      final double velocity = velocitySupplier.getAsDouble();
      drivetrain.drive(velocity, velocity, output, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (distanceTraveled() > 1) {
      drivetrain.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceTraveled() > 1;
  }

  private double distanceTraveled() {
    return Math.sqrt(
        Math.pow((drivetrain.getPoseY() - initialYPose), 2)
            + Math.pow((drivetrain.getPoseX() - initialXPose), 2));
  }
}
