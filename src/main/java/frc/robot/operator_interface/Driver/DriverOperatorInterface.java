// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface.Driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface DriverOperatorInterface {

  default double getTranslateX() {
    return 0.0;
  }

  default double getTranslateY() {
    return 0.0;
  }

  default double getRotate() {
    return 0.0;
  }

  default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  default Trigger getWristNegButton() {
    return new Trigger(() -> false);
  }

  default Trigger getRotateButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempCollectConeFloor() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempCollectConeTip() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempCollectCube() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempCollectCubeGround() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempEject() {
    return new Trigger(() -> false);
  }

  public default Trigger getTempGoHome() {
    return new Trigger(() -> false);
  }

  // CoDriver
  public default Trigger getTempScore() {
    return new Trigger(() -> false);
  }
}
