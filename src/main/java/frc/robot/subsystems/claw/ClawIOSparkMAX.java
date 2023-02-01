package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;

import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClawIOSparkMAX implements ClawIO {

  private CANSparkMax rollerMotor;

  public ClawIOSparkMAX() {
        rollerMotor =
          new CANSparkMax(
            RobotConfig.getInstance().getClawMotorID(),
            CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.currentAmps = new double[] {rollerMotor.getOutputCurrent()};
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
