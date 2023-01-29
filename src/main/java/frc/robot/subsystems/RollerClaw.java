// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerClaw extends SubsystemBase {
  
  public static RollerClaw instance;

  private final CANSparkMax m_motor = new CANSparkMax(Constants.ROLLER_CLAW_MOTOR_CAN_ID, MotorType.kBrushless);
 
  /** Creates a new RollerClaw. */
  public RollerClaw() {
    instance = this;
    stopRoller();
  }

  public static RollerClaw getInstance() {
    return instance;
  }

  public void setRollerSpeed(final double speed) {
    m_motor.set(speed);
  }

  public void runRollerIn() {
    setRollerSpeed(1);
  }

  public void runRollerOut() {
    setRollerSpeed(-1);
  }

  public void stopRoller() {
    m_motor.stopMotor();
  }

  public double getMotorCurrent() {
    return m_motor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
