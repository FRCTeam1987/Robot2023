package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RotateToDegree extends CommandBase {
  private static final double kP = 0.03; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.01; // PID derivative gain
  private static final double kToleranceDegrees = 2.0; // Tolerance for reaching the desired angle
  private static final double kMaxOutput = 0.5; // Maximum output for the PID controller
  private static final double kTimeoutSeconds = 5.0; // Timeout for the command

  private final double targetAngleDegrees;
  private final Drivetrain swerveDrive;
  private final PIDController pidController;
  private final Timer timer;

  public RotateToDegree(double targetAngleDegrees, Drivetrain swerveDrive) {
    this.targetAngleDegrees = targetAngleDegrees;
    this.swerveDrive = swerveDrive;

    // Create the PID controller
    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(kToleranceDegrees);

    // Create a timer to handle the timeout
    timer = new Timer();

    // Add the required subsystem (SwerveDriveSubsystem) to the command's requirements
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset the timer and start it
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Calculate the output based on the PID controller and the current gyro angle
    double output =
        pidController.calculate(swerveDrive.getRotation().getDegrees(), targetAngleDegrees);

    // Apply the output to the swerve drive subsystem to rotate the robot
    swerveDrive.drive(0.0, 0.0, output, true);
  }

  @Override
  public boolean isFinished() {
    // Return true if the robot has reached the target angle or if the command has timed out
    return pidController.atSetpoint() || timer.hasElapsed(kTimeoutSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the swerve drive rotation when the command ends
    swerveDrive.stop();

    // Stop the timer
    timer.stop();
  }
}
