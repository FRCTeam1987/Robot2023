// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveToPiece extends CommandBase {
  /** Creates a new DriveToPiece. */
  private DoubleSupplier velocitySupplier;

  private final Drivetrain drivetrain;

  private final GamePiece gamePiece;

  private static final String limelight = "limelight-collect";

  private Pose2d initialPose;
  private static final double kP = 0.03; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.01; // PID derivative gain
  private static final double kToleranceDegrees = 0.1; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 1.75; // In Meters

  private final PIDController rotationController;

  public DriveToPiece(
      final Drivetrain drivetrain, final DoubleSupplier velocitySupplier, GamePiece gamePiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocitySupplier = velocitySupplier;
    this.drivetrain = drivetrain;
    this.gamePiece = gamePiece;

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
    drivetrain.disableFieldRelative();

    if (gamePiece == GamePiece.CONE) {
      RobotContainer.setConePipeline();
    } else {
      RobotContainer.setCubePipeline();
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight) == false) {
      drivetrain.stop();
      return;
    }
    double rotationalVelocity =
        rotationController.calculate(LimelightHelpers.getTX(limelight), 0.0);
    drivetrain.drive(velocitySupplier.getAsDouble(), 0.0, rotationalVelocity, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.enableFieldRelative();
    drivetrain.stop();
    // if (isDistanceTraveledTooFar() && !interrupted) {
    //   drivetrain.stop();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar();
  }

  private double distanceTraveled() {
    return drivetrain.getPose().getTranslation().getDistance(initialPose.getTranslation());
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > maximumAllowableDistance;
  }
}
