// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.channels.WritableByteChannel;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;

public class SetWristPosition extends CommandBase {
  private int position;
  private final Wrist m_wrist;
  /** Creates a new SetWristPosition. */
  public SetWristPosition(int position, Wrist wrist) {
    m_wrist = wrist;
    this.position = position;
    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO check the telescope length, if the length is below a value change the exceptible range
    m_wrist.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(m_wrist.getPosition(), position, 100);
  }
}
