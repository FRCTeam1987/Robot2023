// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final double kP = 0.07; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.00; // PID derivative gain
  private static final double kToleranceDegrees = 0.1; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 3.0; // In Meters
  private static final double slowDownDistance = 1.0; // Robot goes half speed once passed

  private final PIDController rotationController;

  private Debouncer canSeePieceDebouncer;
  private static final double DEBOUNCE_TIME = 0.06; // TODO find correct value and change name

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
    System.out.println("DriveToPiece - init");
    // Apply the output to the swerve
    this.initialPose = drivetrain.getPose();
    rotationController.reset();
    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();

    if (gamePiece == GamePiece.CONE) {
      RobotContainer.setConePipeline();
    } else {
      RobotContainer.setCubePipeline();
    }
    canSeePieceDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!canSeePieceDebouncer.calculate(LimelightHelpers.getTV(limelight))) {
      // DriverStation.reportWarning("DriveToPiece Can't see gamePicee", false);
      System.out.println("DriveToPiece Can't see gamePice");
      // drivetrain.stop();
      drivetrain.drive(0.0, 0.0, 0.0, true);
      return;
    }

    // if (LimelightHelpers.getTV(limelight) == false) {
    //   drivetrain.stop();
    //   return;
    // }
    double rotationalVelocity =
        rotationController.calculate(LimelightHelpers.getTX(limelight), 0.0);
    double speed =
        distanceTraveled() > slowDownDistance
            ? velocitySupplier.getAsDouble() / 2.0
            : velocitySupplier.getAsDouble();
    // System.out.println("========================= DriveToPiece Speed: " + speed);

    drivetrain.drive(speed, 0.0, rotationalVelocity, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO If it fails to pickup gamepiece send arm home
    DriverStation.reportWarning("DriveToPiece Command Finished", false);
    System.out.println("DriveToPiece Command Finished");
    drivetrain.enableFieldRelative();
    drivetrain.drive(0.0, 0.0, 0.0, true);
    if (isDistanceTraveledTooFar()) {
      DriverStation.reportWarning("DriveToPiece Drove Too Far", false);
      System.out.println("DriveToPiece Drove Too Far");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar();
  }

  private double distanceTraveled() {
    return Math.abs(
        drivetrain.getPose().getTranslation().getDistance(initialPose.getTranslation()));
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > maximumAllowableDistance;
  }
}
