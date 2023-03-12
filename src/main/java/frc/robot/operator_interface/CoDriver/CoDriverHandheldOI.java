// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface.CoDriver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class CoDriverHandheldOI implements CoDriverOperatorInterface {
  private final XboxController controller;

  public CoDriverHandheldOI(int port) {
    controller = new XboxController(port);
  }

  public Trigger getTempHighScore() {
    return new Trigger(controller::getYButton);
  }

  public Trigger getTempMediumScore() {
    return new Trigger(controller::getXButton);
  }

  public Trigger getTempFloorScore() {
    return new Trigger(controller::getAButton);
  }
}
