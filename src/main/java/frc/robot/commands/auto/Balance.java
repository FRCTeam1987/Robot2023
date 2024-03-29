// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Balance extends PIDCommand {

  final Drivetrain drive;
  /** Creates a new Balance. */
  public Balance(final Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(0.225, 0, 0), // HEARTLAND 0.2, 0 ,0 STAGE 025, 0, 0
        // This should return the measurement
        drive::getPitch,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output ->
            // Use the output here
            drive.drive(-output / 7.0, 0, 0, true));

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drive = drive;
    addRequirements(this.drive);
    this.getController().setSetpoint(0);
    this.getController().setTolerance(0.5, 0.25);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
