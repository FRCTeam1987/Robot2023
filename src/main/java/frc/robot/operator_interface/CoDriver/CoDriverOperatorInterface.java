// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface.CoDriver;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface CoDriverOperatorInterface {

  // CoDriver
  public default Trigger getTempHighScore() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempMediumScore() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempFloorScore() {
    return new Trigger(() -> false);
  }
}
