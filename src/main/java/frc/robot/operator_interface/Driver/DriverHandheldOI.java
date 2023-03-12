// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface.Driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class DriverHandheldOI implements DriverOperatorInterface {
  private final XboxController controller;

  public DriverHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  // @Override
  // public Trigger getFieldRelativeButton() {
  //   return new Trigger(controller::getXButton);
  // }

  @Override
  public Trigger getResetGyroButton() {
    return new Trigger(controller::getBackButton);
  }

  // @Override
  // public Trigger getXStanceButton() {
  //   return new Trigger(controller::getYButton);
  // }

  // @Override
  // public Trigger getWristPosButton() {
  //   return new Trigger(controller::getRightBumper);
  // }

  // @Override
  // public Trigger getWristNegButton() {
  //   return new Trigger(controller::getLeftBumper);
  // }

  // @Override
  // public Trigger getRotateButton() {
  //   return new Trigger(controller::getAButton);
  // }

  @Override
  public Trigger getTempCollectConeFloor() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getTempCollectConeTip() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getTempCollectCube() {
    return new Trigger(controller::getRightBumper);
  }

  public Trigger getTempEject() {
    return new Trigger(controller::getLeftBumper);
  }

  public Trigger getTempGoHome() {
    return new Trigger(controller::getStartButton);
  }

  // public Trigger getTempScore() {
  //   return new Trigger(controller::getBButton);
  // }
}
