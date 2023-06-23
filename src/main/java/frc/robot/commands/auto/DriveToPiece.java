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

  private String limelight = "limelight-fr";  // TODO rename this default so that it makes sense for its new purpose OR use the parameter value
  private double initialXPose;
  private double initialYPose;
  private static final double kP = 0.03; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.01; // PID derivative gain
  private static final double kToleranceDegrees = 2.0; // Tolerance for reaching the desired angle

  private final PIDController pidController;  // TODO rename this - what does this pid controller for?

  public DriveToPiece(
      final Drivetrain drivetrain, final DoubleSupplier velocitySupplier, String limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocitySupplier = velocitySupplier;
    this.drivetrain = drivetrain;
    // TODO if using a parameter, then save the limelight string parameter to this.limelight

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
    // TODO reset the PID controller
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * TODO
     * I like the practice of short circuiting to keep the code more readable. For example:
     * if (isVisible() == false) {
     *  return;
     * }
     * // do something if visible
     */
    if (LimelightHelpers.getTV(limelight)) {
      double output = // TODO name this so that it is clear what the output is used for.
          pidController.calculate(
              drivetrain.getRotation().getDegrees(),
              drivetrain.getRotation().getDegrees() - LimelightHelpers.getTX(limelight));

      /*
       * TODO
       * If choosing to do field-relative, then velocity for x and y need to be separate values. Otherwise, field relative will drive in a diagonal.
       * If making it robot-relative, then a single velocity should be fine. Just enable / disable field relative as needed.
       * We may need to flex this between the two modes for auto vs teleop.
       */
      final double velocity = velocitySupplier.getAsDouble();
      drivetrain.drive(velocity, velocity, output, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (distanceTraveled() > 1) { // TODO reuse this logic
      drivetrain.stop();  // TODO do we want to stop at the end anyways or especially when interruped? since this is for collecting game pieces. prevent further auto steps?
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceTraveled() > 1;  // TODO make this 1 a constant or parameter / should this command run forever and let other commands stop it in a ParallelRaceGroup / teleop?
  }

  private double distanceTraveled() {
    // TODO refactor this to use built in Pose logic / make reusable
    // ex. Translation2d's getDistance()
    return Math.sqrt(
        Math.pow((drivetrain.getPoseY() - initialYPose), 2)
            + Math.pow((drivetrain.getPoseX() - initialXPose), 2));
  }
}
