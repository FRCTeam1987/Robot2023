// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreBalance extends CommandBase {

  final Drivetrain drive;
  double maxGyroAngle = 0.0;
  double startingPose = 0.0;
  /** Creates a new Balance. */
  public PreBalance(final Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drive = drive;
    addRequirements(this.drive);
  }

  private boolean hasDrivenDistance() {
    return Math.abs(drive.getPoseX() - startingPose) > .6;
  }

  @Override
  public void initialize() {
    drive.drive(Math.copySign(.75, drive.getPitch()), 0, 0, true);
    startingPose = drive.getPoseX();
  }

  @Override
  public void execute() {
    if (hasDrivenDistance()) {
      maxGyroAngle = Math.max(Math.abs(drive.getPitch()), maxGyroAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("Ended PreBalance", false);
    drive.stop();
    drive.enableXstance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    DriverStation.reportWarning(
        "Ran IsFinished" + (Util.isWithinTolerance(Math.abs(drive.getPitch()), 0, 2)), false);
    return hasDrivenDistance() && Math.abs(drive.getPitch()) < maxGyroAngle - 1;
  }
}
