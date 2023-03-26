// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToPosition extends PIDCommand {

  final Drivetrain drive;
  final double targetRotaion;
  /** Rotates to a position */
  public RotateToPosition(final Drivetrain drive, final double targetRotaion) {
    super(
        // The controller that the command will use
        new PIDController(0.25, 0, 0),
        // This should return the measurement
        () -> (drive.getRotation().getDegrees()),
        // This should return the setpoint (can also be a constant)
        () -> targetRotaion,
        // This uses the output
        output -> {
          // Use the output here
          {
            drive.drive(0, 0, 1, true); // TODO increase roationvelocity after testing
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drive = drive;
    this.targetRotaion = targetRotaion;
    addRequirements(this.drive);
    this.getController().setSetpoint(targetRotaion);
    this.getController().setTolerance(5, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(drive.getRotation().getDegrees(), targetRotaion, 3.5);
  }
}
