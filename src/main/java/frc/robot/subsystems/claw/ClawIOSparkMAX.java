package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClawIOSparkMAX implements ClawIO {

  private final CANSparkMax rollerMotor;

  public ClawIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2023_COMP:
        rollerMotor =
            new CANSparkMax(
                RobotContainer.getInstance().getConfig().getClawMotorID(),
                CANSparkMax.MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid robot for ClawIOSparkMAX!");
    }
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.currentAmps = new double[] {rollerMotor.getOutputCurrent()};
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
