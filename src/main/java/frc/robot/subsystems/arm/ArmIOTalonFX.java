package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.lib.team6328.util.Alert;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;
  private TalonFX telescopingMotor;
  private AnalogPotentiometer telescopePotentiometer;
  private Alert armWentBeserkAlert =
      new Alert("Attempted to set arm beyond safe range.", Alert.AlertType.ERROR);

  double temporaryArmCancoderOffset = 160;

  public ArmIOTalonFX(
      int leaderMotorID,
      int followerMotorID,
      int rotationCANCoderID,
      int telescopingMotorID,
      String canBusName) {

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.sensorDirection = false;
    rotationEncoder = new CANCoder(rotationCANCoderID, canBusName);
    rotationEncoder.configFactoryDefault();
    rotationEncoder.configAllSettings(canCoderConfiguration);
    rotationEncoder.setPosition(rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    talonFXConfig.remoteFilter0.remoteSensorDeviceID = rotationCANCoderID;
    talonFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    talonFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    talonFXConfig.slot0.kP = 5;
    talonFXConfig.slot0.kI = 0.0;
    talonFXConfig.slot0.kD = 0.0;
    talonFXConfig.slot0.kF = 0.0;
    talonFXConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    talonFXConfig.feedbackNotContinuous = true;
    talonFXConfig.slot0.allowableClosedloopError = 0;
    rotationLeader = new TalonFX(leaderMotorID, canBusName);
    rotationLeader.configFactoryDefault();
    rotationLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    rotationLeader.configAllSettings(talonFXConfig);
    rotationFollower = new TalonFX(followerMotorID, canBusName);
    rotationFollower.configFactoryDefault();
    rotationFollower.follow(rotationLeader);

    telescopingMotor = new TalonFX(telescopingMotorID);
    telescopingMotor.configFactoryDefault();

    telescopePotentiometer = new AnalogPotentiometer(4, 20, 0);
  }

  public int convertDegreesToTicks(double deg) {
    return (int) (deg * 4096) / 360;
  }

  public double convertTicksToDegrees(double ticks) {
    return 360 * (ticks / 4096);
  }

  public double getArmLength() {
    try {
      return telescopePotentiometer.get();
    } catch (Exception e)  {
      return 9.9;
    }
    
    // TODO: make this work
  }

  @Override
  public double getEncoderPosition() {
    return rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset;
  }

  @Override
  public double getEncoderPositionNoOffset() {
    return rotationEncoder.getAbsolutePosition();
  }

  @Override
  public double getTalonPosition() {
    return rotationLeader.getSelectedSensorPosition();
  }

  public void extendArmToLength(double length) {
    telescopingMotor.set(TalonFXControlMode.Position, length);
  }

  @Override
  public void rotateArmToAngle(double angle) {
    if (!(Math.abs(angle) > 120)) {
      rotationLeader.set(TalonFXControlMode.Position, convertDegreesToTicks(angle));
    } else {
      armWentBeserkAlert.set(true);
    }
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps =
        new double[] {
          rotationLeader.getStatorCurrent(),
          rotationFollower.getStatorCurrent(),
          telescopingMotor.getStatorCurrent()
        };
    inputs.currentVolts =
        new double[] {
          rotationLeader.getMotorOutputVoltage(),
          rotationFollower.getMotorOutputVoltage(),
          telescopingMotor.getMotorOutputVoltage()
        };
    inputs.armAbsoluteAngle = getEncoderPosition();
    inputs.armLength =
        new double[] {
          this.getArmLength()
        };
  }
}
