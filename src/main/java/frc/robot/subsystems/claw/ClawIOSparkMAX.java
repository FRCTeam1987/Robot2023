package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;

public class ClawIOSparkMAX implements ClawIO {

  private CANSparkMax rollerMotor;

  public ClawIOSparkMAX() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor =
        new CANSparkMax(
            RobotConfig.getInstance().getClawMotorID(), CANSparkMax.MotorType.kBrushless);
    
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.currentAmps = rollerMotor.getOutputCurrent();
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
