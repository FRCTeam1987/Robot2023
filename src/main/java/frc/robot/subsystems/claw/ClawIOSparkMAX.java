package frc.robot.subsystems.claw;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

public class ClawIOSparkMAX implements ClawIO {

  private final CANSparkMax rollerMotor;

  public ClawIOSparkMAX() {
    rollerMotor =
        new CANSparkMax(
            RobotConfig.getInstance().getClawMotorID(), CANSparkMax.MotorType.kBrushless);
    REVLibError restoreError = rollerMotor.restoreFactoryDefaults();
    if (restoreError != REVLibError.kOk) {
      new Alert(restoreError.toString(), AlertType.WARNING);
    }
    REVLibError flashError = rollerMotor.burnFlash();
    if (flashError != REVLibError.kOk) {
      new Alert(flashError.toString(), AlertType.WARNING);
    }
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    double amps = 0.0;
    try {
      amps = rollerMotor.getOutputCurrent();
    } catch (Exception e) {
      new Alert(e.getMessage(), AlertType.WARNING);
      System.out.print(e.getMessage());
      System.out.print("error here");

    }
    if (amps > 0) {
      inputs.currentAmps = amps;
    } else {
      inputs.currentAmps = 0.0;
    }
    
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
