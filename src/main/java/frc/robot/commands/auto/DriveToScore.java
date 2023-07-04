// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveToScore extends CommandBase {

  private static final String LIMELIGHT_NAME = RobotContainer.LIMELIGHT_SCORE;
  private static final double TY_CONES = 4.5;
  private static final double TY_CUBES = 3.5;
  private static final double TY_MAGIC_OFFSET = 11.5; // Avoid irregular roll over of zero

  private final Drivetrain m_drive;
  private final Claw m_claw;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_thetaController;

  /** Creates a new DriveToScore. */
  public DriveToScore(final Drivetrain drive, final Claw claw) {
    m_drive = drive;
    m_claw = claw;
    m_xController = new PIDController(0.1, 0, 0);
    m_xController.setTolerance(2.5);

    m_yController = new PIDController(0.075, 0, 0);
    m_yController.setTolerance(3.0);
    m_yController.enableContinuousInput(-28.5, 28.5);
    m_yController.setSetpoint(0);

    m_thetaController = new PIDController(0.03, 0.0, 0.0);
    m_thetaController.setTolerance(4.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setSetpoint(180);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("DriveToScore Command Started", false);
    m_drive.enableFieldRelative();
    m_xController.setSetpoint((m_claw.isCone() ? TY_CONES : TY_CUBES) + TY_MAGIC_OFFSET);
    m_xController.reset();
    m_yController.reset();
    m_thetaController.reset();
    if (m_claw.isCone()) {
      RobotContainer.setRetroReflectPipeline();
    } else {
      RobotContainer.setAprilTagPipeline();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.canSeeScoringTarget() == false) {
      DriverStation.reportWarning("DriveToScore Can't see target", false);

      return;
    }

    // System.out.println("_____x: " +m_xController.calculate(LimelightHelpers.getTY(LIMELIGHT_NAME)
    // + TY_MAGIC_OFFSET) + "y: " +
    // m_yController.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME)));
    System.out.println(
        "ty: "
            + (LimelightHelpers.getTY(LIMELIGHT_NAME) + TY_MAGIC_OFFSET)
            + " tx: "
            + LimelightHelpers.getTX(LIMELIGHT_NAME));
    m_drive.drive(
        -m_xController.calculate(LimelightHelpers.getTY(LIMELIGHT_NAME) + TY_MAGIC_OFFSET),
        -Math.min(m_yController.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME)), 0.4),
        m_thetaController.calculate(m_drive.getPose().getRotation().getDegrees()),
        true);

    // m_drive.drive(
    //     -MathUtil.clamp(m_xController.calculate(LimelightHelpers.getTY(LIMELIGHT_NAME) +
    // TY_MAGIC_OFFSET), 0.02, 0.5),
    //     -MathUtil.clamp(m_yController.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME)), 0.2,
    // 0.5),
    //     m_thetaController.calculate(m_drive.getPose().getRotation().getDegrees()),
    //     true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("DriveToScore at All SetPoints, command endings", false);
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atSetpoint()
        && m_yController.atSetpoint()
        && m_thetaController.atSetpoint();
  }
}
