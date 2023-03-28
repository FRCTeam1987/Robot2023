// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

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

  @Override
  public void initialize() {
    drive.drive(Math.copySign(0.75, drive.getPitch()), 0, 0, true);
    maxGyroAngle = Math.abs(drive.getPitch());
    startingPose = drive.getPoseX();
  }

  @Override
  public void execute() {

    maxGyroAngle = Math.max(Math.abs(drive.getPitch()), maxGyroAngle);
    System.out.println("MaxAngle " + maxGyroAngle);
    System.out.println(drive.getPitch());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended");
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(
        "Ran IsFinished" + (Util.isWithinTolerance(Math.abs(drive.getPitch()), 0, 2)));
    return Math.abs(drive.getPoseX() - startingPose) > 0.75
        && Math.abs(drive.getPitch()) < maxGyroAngle - 2;

    // return Util.isWithinTolerance(Math.abs(drive.getPitch()), 0, 1);
  }
}
