package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.commands.arm.SetArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveToObject extends CommandBase {
  private Drivetrain drivetrain;
  private Arm arm;
  private double driveSpeed;
  private PIDController rotationController;
  private String limelight = "limelight-fr";

  public DriveToObject(String limelight, Drivetrain drivetrain, Arm arm, double driveSpeed) {
    addRequirements(drivetrain, arm);
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.driveSpeed = driveSpeed;

    double kP = 0.01; // Tune the PID constants as per your requirements
    double kI = 0.0;
    double kD = 0.0;
    rotationController = new PIDController(kP, kI, kD);
    rotationController.setTolerance(5); // degree tolerance
  }

  @Override
  public void initialize() {
    new SetArm(
        arm, () -> 0.0, () -> 1.0, () -> true); // Reset the arm rotation to its initial position
    if (LimelightHelpers.getTV(limelight)) {
      new RotateToDegree(
          drivetrain.getRotation().getDegrees() - LimelightHelpers.getTX(limelight), drivetrain);
    }
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight)) {
      double tA = LimelightHelpers.getTA(limelight);

      if (tA > 0.6) { // Guess value for Standing cone, TODO replace with proper value
        // Rotate the arm when the target distance is reached
        new SetArm(arm, () -> 45.0, () -> 4.0, () -> false); // TODO Set proper values
      } else if (tA > 0.2
          && tA < 0.3) { // Guess value for fallen cone, TODO replace with proper value
        // Rotate the arm when the target distance is reached
        new SetArm(arm, () -> -45.0, () -> 7.0, () -> false); // TODO Set proper values
      } else { // Saftey
        arm.stop();
      }

      double driveCommand = driveSpeed;
      drivetrain.drive(driveCommand, driveCommand, 0.0, true);

    } else {
      drivetrain.stop(); // Safety, if no object is seen, the robot should stop
    }
  }

  @Override
  public boolean isFinished() {
    return !LimelightHelpers.getTV(limelight);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop(); // Stay on stage
    arm.stallArm();
  }
}
