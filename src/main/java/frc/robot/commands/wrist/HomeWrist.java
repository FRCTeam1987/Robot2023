// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.Wrist;

public class HomeWrist extends CommandBase { // README not tested do not use

  private static final double HOMING_PERCENT = -0.45;
  // private static final double HOMING_CURRENT_THRESHOLD = 5;

  private final Wrist wrist;

  /** Creates a new HomeWrist. */
  public HomeWrist(final Wrist wrist) {
    this.wrist = wrist;
    addRequirements(this.wrist);
    // Constants.TAB_WRIST.add(
    //     "ORDRel",
    //     new InstantCommand(() ->
    // wrist.configRelative(Constants.INSTALLED_ARM.getWristOffset())));
    // Constants.TAB_WRIST.add(
    //     "Straightenify", new InstantCommand(() -> wrist.setPosition(Wrist.ANGLE_STRAIGHT)));
    Constants.TAB_MAIN2.addNumber("WristPos", wrist::getPosition).withPosition(9, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPercent(HOMING_PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Interrupted: " + interrupted);
    if (interrupted) {
      return;
    }
    wrist.setPercent(0);
    System.out.println(wrist.getPosition() + " Before ConfigRelative");
    wrist.configRelative(Constants.INSTALLED_ARM.getWristOffset());
    System.out.println(wrist.getPosition() + " After ConfigRelative");
    wrist.setPosition(Wrist.ANGLE_STRAIGHT);
    System.out.println(wrist.getPosition() + " After SetPosition");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("Wrist Current:" + wrist.getCurrent());
    System.out.println(
        "is Finished: " + wrist.hasHitHardstop() + ", position: " + wrist.getPosition());
    return wrist.hasHitHardstop();

    // return Math.abs(wrist.getCurrent()) > HOMING_CURRENT_THRESHOLD;
  }
}
